#!/usr/bin/env python3
"""
Eagle CAD Schematic Generator from AI MD Files
Converts markdown schematic files to Eagle CAD .SCR scripts and .SCH files
"""

__version__ = "1.4.10"

import os
import re
import json
from datetime import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Set
import xml.etree.ElementTree as ET


CONFIG_FILE = Path(__file__).with_name("eagle_schematic_config.txt")
LIB_CACHE_FILE = Path(__file__).with_name("eagle_library_cache.json")
NC_IDENTIFIER_TOKENS = {
    'NC',
    'NOCONNECT',
    'NOTCONNECTED',
    'NOCONNECTION',
    'NOTCONNECT',
    'DONOTCONNECT',
    'DONTCONNECT',
    'DNC',
    'OPEN',
    'FLOAT'
}


def normalize_identifier(value: Optional[str]) -> str:
    if value is None:
        return ''
    normalized = re.sub(r'[^A-Z0-9]+', '', str(value).upper())
    return normalized


def normalize_component_value(value: str) -> str:
    """Normalize encoding-corrupted symbols in component values (Ω, µ, etc.)."""
    if not value:
        return value
    # Fix ? used as ohm symbol (e.g. 15k?, 100?, 1.2k?)
    value = re.sub(r'(\d+(?:\.\d+)?)\s*([kKmM]?)\?', r'\1\2Ω', value)
    # Fix u used as micro in capacitance (e.g. 0.1uF -> 0.1µF) - optional, uF is commonly accepted
    # Leave uF as-is since it's valid; only fix encoding issues
    return value


class WireSegmentRegistry:
    """
    Unified tracking system for all wire segments to prevent overlaps.
    
    This class maintains a spatial index of all placed wire segments and provides
    collision detection for routing. It ensures that:
    1. No two wire segments from different nets overlap or get too close
    2. Horizontal and vertical channels are uniquely assigned
    3. Junctions only occur at explicit connection points
    """
    
    def __init__(self, grid_step: float = 2.54, min_clearance: float = 2.54):
        self.grid_step = grid_step
        # CRITICAL: Much larger clearance to prevent "close but unconnected" ERC warnings
        # Eagle's default wire-to-wire clearance check is about 0.01" (0.254mm)
        # We need at least 10x that to be safe
        self.min_clearance = max(min_clearance, grid_step * 4.0, 10.0)  # At least 10mm clearance
        
        # Track all wire segments: list of (net_id, x1, y1, x2, y2)
        self.segments: List[Tuple[str, float, float, float, float]] = []
        
        # Track horizontal channels: {y_coord: [(net_id, x_start, x_end), ...]}
        self.h_channels: Dict[float, List[Tuple[str, float, float]]] = {}
        
        # Track vertical channels: {x_coord: [(net_id, y_start, y_end), ...]}
        self.v_channels: Dict[float, List[Tuple[str, float, float]]] = {}
        
        # Reserved channels for specific nets: {net_id: y_coord}
        self.net_h_lanes: Dict[str, float] = {}
        
        # Track junction points: set of (x, y) where connections are allowed
        self.junction_points: Set[Tuple[float, float]] = set()
        
        # Component keep-out zones: list of (x_min, y_min, x_max, y_max)
        self.keepout_zones: List[Tuple[float, float, float, float]] = []
        
        # All pin Y-coordinates for avoiding routing through pin rows
        self.pin_y_coords: Set[float] = set()
        self.pin_x_coords: Set[float] = set()
        
    def snap(self, value: float) -> float:
        """Snap coordinate to grid."""
        step = self.grid_step / 10.0  # Fine routing grid
        return round(value / step) * step
    
    def add_keepout_zone(self, center_x: float, center_y: float, half_w: float, half_h: float):
        """Register a rectangular area where routing should not pass through."""
        x_min = center_x - half_w - self.min_clearance
        y_min = center_y - half_h - self.min_clearance
        x_max = center_x + half_w + self.min_clearance
        y_max = center_y + half_h + self.min_clearance
        self.keepout_zones.append((x_min, y_min, x_max, y_max))
    
    def add_pin_location(self, x: float, y: float):
        """Register a pin location for clearance checking."""
        self.pin_y_coords.add(round(y, 2))
        self.pin_x_coords.add(round(x, 2))
    
    def register_segment(self, net_id: str, x1: float, y1: float, x2: float, y2: float):
        """Register a wire segment for collision tracking."""
        x1, y1 = self.snap(x1), self.snap(y1)
        x2, y2 = self.snap(x2), self.snap(y2)
        
        self.segments.append((net_id, x1, y1, x2, y2))
        
        # Track in appropriate channel index
        if abs(y1 - y2) < 0.01:  # Horizontal segment
            y = round(y1, 2)
            x_min, x_max = min(x1, x2), max(x1, x2)
            if y not in self.h_channels:
                self.h_channels[y] = []
            self.h_channels[y].append((net_id, x_min, x_max))
        elif abs(x1 - x2) < 0.01:  # Vertical segment
            x = round(x1, 2)
            y_min, y_max = min(y1, y2), max(y1, y2)
            if x not in self.v_channels:
                self.v_channels[x] = []
            self.v_channels[x].append((net_id, y_min, y_max))
    
    def add_junction(self, x: float, y: float):
        """Mark a point where wire connections are allowed."""
        self.junction_points.add((round(x, 2), round(y, 2)))
    
    def _ranges_overlap(self, a_min: float, a_max: float, b_min: float, b_max: float, margin: float = 0) -> bool:
        """Check if two 1D ranges overlap with optional margin."""
        return not (a_max + margin < b_min or b_max + margin < a_min)
    
    def _check_horizontal_collision(self, net_id: str, y: float, x_start: float, x_end: float) -> bool:
        """Check if a horizontal segment at y from x_start to x_end would collide with existing segments."""
        y_key = round(y, 2)
        x_min, x_max = min(x_start, x_end), max(x_start, x_end)
        
        # Check against existing horizontal segments at same or nearby Y
        for existing_y, segments in self.h_channels.items():
            if abs(existing_y - y_key) < self.min_clearance:
                for seg_net, seg_x_min, seg_x_max in segments:
                    if seg_net != net_id:  # Same net can overlap
                        if self._ranges_overlap(x_min, x_max, seg_x_min, seg_x_max, self.min_clearance):
                            return True
        
        # Check against vertical segments that cross this Y level
        for existing_x, segments in self.v_channels.items():
            if x_min - self.min_clearance <= existing_x <= x_max + self.min_clearance:
                for seg_net, seg_y_min, seg_y_max in segments:
                    if seg_net != net_id:
                        # Check if vertical segment crosses our Y level
                        if seg_y_min - self.min_clearance <= y_key <= seg_y_max + self.min_clearance:
                            return True
        
        return False
    
    def _check_vertical_collision(self, net_id: str, x: float, y_start: float, y_end: float) -> bool:
        """Check if a vertical segment at x from y_start to y_end would collide."""
        x_key = round(x, 2)
        y_min, y_max = min(y_start, y_end), max(y_start, y_end)
        
        # Check against existing vertical segments at same or nearby X
        for existing_x, segments in self.v_channels.items():
            if abs(existing_x - x_key) < self.min_clearance:
                for seg_net, seg_y_min, seg_y_max in segments:
                    if seg_net != net_id:
                        if self._ranges_overlap(y_min, y_max, seg_y_min, seg_y_max, self.min_clearance):
                            return True
        
        # Check against horizontal segments that cross this X level
        for existing_y, segments in self.h_channels.items():
            if y_min - self.min_clearance <= existing_y <= y_max + self.min_clearance:
                for seg_net, seg_x_min, seg_x_max in segments:
                    if seg_net != net_id:
                        if seg_x_min - self.min_clearance <= x_key <= seg_x_max + self.min_clearance:
                            return True
        
        return False
    
    def _crosses_keepout(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Check if a segment would cross any keepout zone."""
        for (kx_min, ky_min, kx_max, ky_max) in self.keepout_zones:
            # Check if segment intersects keepout rectangle
            seg_x_min, seg_x_max = min(x1, x2), max(x1, x2)
            seg_y_min, seg_y_max = min(y1, y2), max(y1, y2)
            
            if (seg_x_max >= kx_min and seg_x_min <= kx_max and
                seg_y_max >= ky_min and seg_y_min <= ky_max):
                return True
        return False
    
    def _too_close_to_pins(self, y: float, is_horizontal: bool = True) -> bool:
        """Check if a routing Y coordinate is too close to pin rows."""
        if is_horizontal:
            for pin_y in self.pin_y_coords:
                if abs(y - pin_y) < self.min_clearance * 2:
                    return True
        return False
    
    def find_free_horizontal_lane(self, net_id: str, base_y: float, x_start: float, x_end: float,
                                   search_range: float = 500.0) -> float:
        """Find a clear horizontal lane for routing."""
        # Check if net already has an assigned lane
        if net_id in self.net_h_lanes:
            existing_y = self.net_h_lanes[net_id]
            if not self._check_horizontal_collision(net_id, existing_y, x_start, x_end):
                return existing_y
        
        base_y = self.snap(base_y)
        # Use large step to ensure good separation between lanes
        step = max(self.grid_step * 3.0, self.min_clearance * 2.0, 15.0)
        
        # Search in expanding rings from base_y
        for distance in range(int(search_range / step) + 1):
            for direction in [1, -1]:
                candidate = self.snap(base_y + direction * distance * step)
                
                # Skip if too close to pin rows
                if self._too_close_to_pins(candidate):
                    continue
                
                # Skip if would cross keepout zone
                if self._crosses_keepout(x_start, candidate, x_end, candidate):
                    continue
                
                # Check for collisions
                if not self._check_horizontal_collision(net_id, candidate, x_start, x_end):
                    self.net_h_lanes[net_id] = candidate
                    return candidate
        
        # Fallback: return a lane very far away
        fallback = self.snap(base_y - search_range * 2)
        self.net_h_lanes[net_id] = fallback
        return fallback
    
    def find_free_vertical_column(self, net_id: str, base_x: float, y_start: float, y_end: float,
                                   search_range: float = 200.0) -> float:
        """Find a clear vertical column for routing."""
        base_x = self.snap(base_x)
        # Use large step to ensure good separation between columns
        step = max(self.grid_step * 3.0, self.min_clearance * 2.0, 15.0)
        
        for distance in range(int(search_range / step) + 1):
            for direction in [1, -1]:
                candidate = self.snap(base_x + direction * distance * step)
                
                # Skip if would cross keepout zone
                if self._crosses_keepout(candidate, y_start, candidate, y_end):
                    continue
                
                if not self._check_vertical_collision(net_id, candidate, y_start, y_end):
                    return candidate
        
        return self.snap(base_x - search_range * 2)
    
    def can_route_direct(self, net_id: str, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Check if a direct orthogonal route is possible."""
        if abs(y1 - y2) < 0.01:  # Horizontal
            return not self._check_horizontal_collision(net_id, y1, x1, x2)
        elif abs(x1 - x2) < 0.01:  # Vertical
            return not self._check_vertical_collision(net_id, x1, y1, y2)
        return False


def is_nc_identifier(value: Optional[str]) -> bool:
    normalized = normalize_identifier(value)
    if not normalized:
        return False
    if normalized in NC_IDENTIFIER_TOKENS:
        return True
    if normalized.startswith('NC'):
        return True
    return False
class EagleLibraryParser:
    """Parse Eagle CAD library files to find components"""

    CACHE_VERSION = 2

    def __init__(self, eagle_dir: str):
        self.eagle_dir = Path(eagle_dir)
        self.library_cache = {}  # Full XML trees keyed by library name
        self.device_map: Dict[str, List[Dict]] = {}
        self.part_index: Dict[str, List[Dict]] = {}
        self.manual_mappings = {}  # User-defined component mappings
        self.library_xml_cache = {}  # Cached library XML roots for embedding
        self.library_file_metadata: List[Dict[str, float]] = []
        self.device_count: int = 0
        self.cache_loaded = False

    def _store_device_entry(self, key: str, entry: Dict):
        """Store a component entry under the provided lookup key."""
        if key not in self.device_map:
            self.device_map[key] = []
        self.device_map[key].append(entry)

    def _index_entry_fields(self, entry: Dict):
        fields = (
            entry.get('deviceset', ''),
            entry.get('device', ''),
            entry.get('package', '')
        )
        for field in fields:
            if not field:
                continue
            norm = self._normalize_component_name(field)
            if not norm:
                continue
            self.part_index.setdefault(norm, []).append(entry)

    def _rebuild_part_index(self, entries: List[Dict]):
        self.part_index.clear()
        for entry in entries:
            self._index_entry_fields(entry)

    def _iter_unique_devices(self):
        """Iterate over each unique component entry once."""
        seen = set()
        for entry_list in self.device_map.values():
            for entry in entry_list:
                entry_id = id(entry)
                if entry_id in seen:
                    continue
                seen.add(entry_id)
                yield entry

    def _cache_key(self) -> str:
        try:
            return str(self.eagle_dir.resolve())
        except Exception:
            return str(self.eagle_dir)

    def _normalize_path_str(self, path: Path) -> str:
        try:
            return str(path.resolve())
        except Exception:
            return str(path)

    def _collect_library_files(self) -> List[Path]:
        files: List[Path] = []
        root = self._normalize_path_str(self.eagle_dir)
        if not os.path.isdir(root):
            return files
        for dirpath, _, filenames in os.walk(root):
            for name in filenames:
                if name.lower().endswith('.lbr'):
                    files.append(Path(dirpath) / name)
        files.sort()
        return files

    def _build_library_metadata(self, files: List[Path]) -> List[Dict[str, float]]:
        metadata: List[Dict[str, float]] = []
        for path in files:
            try:
                stat = path.stat()
                mtime = stat.st_mtime
                size = stat.st_size
            except OSError:
                mtime = 0
                size = 0
            metadata.append({
                'path': self._normalize_path_str(path),
                'name': path.name,
                'mtime': mtime,
                'size': size
            })
        return metadata

    def _count_unique_devices(self) -> int:
        seen = set()
        count = 0
        for entry_list in self.device_map.values():
            for entry in entry_list:
                entry_id = id(entry)
                if entry_id in seen:
                    continue
                seen.add(entry_id)
                count += 1
        return count

    def _sanitize_entry_for_cache(self, entry: Dict) -> Dict:
        sanitized = dict(entry)
        sanitized['deviceset_elem'] = None
        sanitized['device_elem'] = None
        pins = entry.get('pins') or {}
        sanitized['pins'] = {
            pin: dict(data)
            for pin, data in pins.items()
        }
        sanitized['gate_names'] = list(entry.get('gate_names', []))
        return sanitized

    def _serialize_device_map(self) -> Tuple[List[Dict], Dict[str, List[int]]]:
        entries: List[Dict] = []
        entry_index: Dict[int, int] = {}
        for entry in self._iter_unique_devices():
            sanitized = self._sanitize_entry_for_cache(entry)
            idx = len(entries)
            entries.append(sanitized)
            entry_index[id(entry)] = idx

        key_map: Dict[str, List[int]] = {}
        for key, entry_list in self.device_map.items():
            key_map[key] = [entry_index[id(entry)] for entry in entry_list]
        return entries, key_map

    def _rebuild_device_map_from_cache(self, entries: List[Dict], key_index: Dict[str, List[int]]):
        entry_objects = [dict(entry) for entry in entries]
        rebuilt: Dict[str, List[Dict]] = {}
        for key, indices in key_index.items():
            rebuilt[key] = [entry_objects[idx] for idx in indices if 0 <= idx < len(entry_objects)]
        self.device_map = rebuilt
        self._rebuild_part_index(entry_objects)

    def ensure_library_pins(self, library_name: str) -> bool:
        """Deep-scan a specific library on demand so pin data is available."""
        lib_name = (library_name or '').strip()
        if not lib_name:
            return False
        if lib_name in self.library_xml_cache:
            return True
        try:
            lbr_file = None
            target = lib_name.lower()
            for meta in self.library_file_metadata:
                name_only = (meta.get('name') or '').rsplit('.', 1)[0]
                if name_only.lower() == target:
                    lbr_file = Path(meta['path'])
                    break
            if lbr_file is None:
                for candidate in self.eagle_dir.rglob(f"{lib_name}.lbr"):
                    lbr_file = candidate
                    break
            if not lbr_file:
                print(f"Unable to locate library file for {library_name}")
                return False
            self._parse_library_deep(lbr_file)
            return True
        except Exception as exc:
            print(f"Error deep scanning {library_name}: {exc}")
            return False

    def _validate_cache_file_state(self, cached_files: List[Dict], current_files: List[Path]) -> Tuple[bool, str]:
        current_lookup: Dict[str, Dict[str, float]] = {}
        for path in current_files:
            key = self._normalize_path_str(path)
            try:
                stat = path.stat()
            except OSError as exc:
                return False, f"Unable to stat {path.name}: {exc}"
            current_lookup[key] = {
                'mtime': stat.st_mtime,
                'size': stat.st_size,
                'name': path.name
            }

        cached_lookup = {item.get('path'): item for item in cached_files}
        if len(current_lookup) != len(cached_lookup):
            return False, "Library file count changed"

        for path_key, cached in cached_lookup.items():
            current = current_lookup.get(path_key)
            if not current:
                name = cached.get('name', path_key)
                return False, f"Library missing: {name}"
            if abs(current['mtime'] - cached.get('mtime', 0)) > 1e-3 or int(current['size']) != int(cached.get('size', -1)):
                name = cached.get('name', path_key)
                return False, f"Library changed: {name}"

        return True, ""

    def load_cache(self, cache_file: Path) -> Tuple[bool, str]:
        if not cache_file.exists():
            return False, f"No cache file at {cache_file.name}"
        try:
            with cache_file.open('r', encoding='utf-8') as f:
                cache_data = json.load(f)
        except Exception as exc:
            return False, f"Unable to read cache: {exc}"

        if cache_data.get('version') != self.CACHE_VERSION:
            return False, "Cache version mismatch"

        caches = cache_data.get('caches', {})
        cache_entry = caches.get(self._cache_key())
        if not cache_entry:
            return False, "No cache entry for this Eagle directory"

        cached_files = cache_entry.get('library_files', [])
        current_files = self._collect_library_files()
        valid, reason = self._validate_cache_file_state(cached_files, current_files)
        if not valid:
            return False, f"Cache invalid: {reason}"

        entries = cache_entry.get('device_entries')
        key_index = cache_entry.get('key_index')
        if not entries or key_index is None:
            return False, "Cache missing device map"

        self._rebuild_device_map_from_cache(entries, key_index)
        self.library_file_metadata = cached_files
        self.device_count = cache_entry.get('device_count', len(entries))
        self.cache_loaded = True
        generated = cache_entry.get('generated_at', 'unknown time')
        return True, f"Loaded {self.device_count} cached device entries (generated {generated})"

    def save_cache(self, cache_file: Path) -> Tuple[bool, str]:
        try:
            entries, key_index = self._serialize_device_map()
            payload = {
                'generated_at': datetime.utcnow().isoformat() + 'Z',
                'device_entries': entries,
                'key_index': key_index,
                'device_count': len(entries),
                'library_files': self.library_file_metadata
            }

            if cache_file.exists():
                try:
                    with cache_file.open('r', encoding='utf-8') as f:
                        cache_data = json.load(f)
                except Exception:
                    cache_data = {'version': self.CACHE_VERSION, 'caches': {}}
            else:
                cache_data = {'version': self.CACHE_VERSION, 'caches': {}}

            cache_data['version'] = self.CACHE_VERSION
            cache_data.setdefault('caches', {})
            cache_data['caches'][self._cache_key()] = payload

            with cache_file.open('w', encoding='utf-8') as f:
                json.dump(cache_data, f, indent=2)

            return True, f"Cached {len(entries)} devices to {cache_file.name}"
        except Exception as exc:
            return False, f"Warning: could not save cache ({exc})"

    def get_device_pins(self, library: str, deviceset: str, device: str = "") -> List[str]:
        """Return sorted pin names for a specific library/device combination."""
        pins = set()
        for entry_list in self.device_map.values():
            for entry in entry_list:
                if (entry.get('library') == library and
                        entry.get('deviceset') == deviceset and
                        entry.get('device') == device):
                    pins.update(entry.get('pins', {}).keys())
        if pins:
            return sorted(pins)
        pins.update(self._get_library_pin_names(library, deviceset, device))
        return sorted(pins)

    def _get_library_pin_names(self, library: str, deviceset: str, device: str = "") -> List[str]:
        """Extract pin names directly from cached library XML."""
        library = (library or '').strip()
        deviceset = (deviceset or '').strip()
        device = (device or '').strip()

        if not deviceset:
            return []

        lib_elem = self.library_xml_cache.get(library)
        if lib_elem is None and library:
            target = library.lower()
            for name, elem in self.library_xml_cache.items():
                if name.lower() == target:
                    lib_elem = elem
                    break
        if lib_elem is None:
            for elem in self.library_xml_cache.values():
                probe = elem.find(f".//deviceset[@name='{deviceset}']")
                if probe is not None:
                    lib_elem = elem
                    break
        if lib_elem is None:
            return []
        deviceset_elem = lib_elem.find(f".//deviceset[@name='{deviceset}']")
        if deviceset_elem is None:
            for candidate in lib_elem.findall('.//deviceset'):
                if candidate.get('name', '').lower() == deviceset.lower():
                    deviceset_elem = candidate
                    break
        if deviceset_elem is None:
            return []

        pins = set()

        device_elems = deviceset_elem.findall('.//device')
        if device:
            for dev_elem in device_elems:
                candidate_name = dev_elem.get('name', '')
                if candidate_name == device or candidate_name.lower() == device.lower():
                    for connect in dev_elem.findall('.//connect'):
                        pin_name = connect.get('pin')
                        if pin_name:
                            pins.add(pin_name)
                    break
        if not pins:
            for dev_elem in device_elems:
                for connect in dev_elem.findall('.//connect'):
                    pin_name = connect.get('pin')
                    if pin_name:
                        pins.add(pin_name)

        if pins:
            return sorted(pins)

        # As a last resort, pull pin names from the associated symbol
        gate_elems = deviceset_elem.findall('.//gate')
        symbol_names = {gate.get('symbol') for gate in gate_elems if gate.get('symbol')}
        lowered = {name.lower() for name in symbol_names}
        for symbol_elem in lib_elem.findall('.//symbol'):
            symbol_name = symbol_elem.get('name', '')
            if not symbol_name:
                continue
            if symbol_name not in symbol_names and symbol_name.lower() not in lowered:
                continue
            for pin in symbol_elem.findall('.//pin'):
                pin_name = pin.get('name')
                if pin_name:
                    pins.add(pin_name)

        return sorted(pins)

    def load_manual_mappings(self, mapping_file: str):
        """Load manual component mappings from JSON file"""
        try:
            with open(mapping_file, 'r') as f:
                self.manual_mappings = json.load(f)
            return True, f"Loaded {len(self.manual_mappings)} manual mappings"
        except FileNotFoundError:
            return False, "Mapping file not found"
        except json.JSONDecodeError as e:
            return False, f"Invalid JSON: {e}"

    def save_manual_mappings(self, mapping_file: str):
        """Save manual component mappings to JSON file"""
        try:
            with open(mapping_file, 'w') as f:
                json.dump(self.manual_mappings, f, indent=2)
            return True, "Mappings saved successfully"
        except Exception as e:
            return False, f"Error saving mappings: {e}"

    def add_manual_mapping(self, ref_prefix: str, library: str, deviceset: str, device: str = ""):
        """Add a manual component mapping"""
        self.manual_mappings[ref_prefix] = {
            'library': library,
            'deviceset': deviceset,
            'device': device
        }

    def scan_libraries(self, progress_callback=None):

        """Quick scan: only extract component names for matching (no pin data)"""
        self.cache_loaded = False
        lbr_files = self._collect_library_files()
        total = len(lbr_files)
        self.library_file_metadata = self._build_library_metadata(lbr_files)

        for idx, lbr_file in enumerate(lbr_files):
            if progress_callback:
                progress_callback(idx + 1, total, lbr_file.name)
            try:
                self._parse_library_quick(lbr_file)
            except Exception as e:
                print(f"Error parsing {lbr_file.name}: {e}")

        self.device_count = self._count_unique_devices()
        return self.device_count

    def deep_scan_used_libraries(self, used_libraries: set, progress_callback=None):
        """Deep scan: extract pin coordinates and cache XML for used libraries only"""
        total = len(used_libraries)

        for idx, lib_name in enumerate(used_libraries):
            if progress_callback:
                progress_callback(idx + 1, total, f"Deep scanning {lib_name}")

            # Find the .lbr file
            lbr_file = None
            for path in self.eagle_dir.rglob(f"{lib_name}.lbr"):
                lbr_file = path
                break

            if lbr_file:
                try:
                    self._parse_library_deep(lbr_file)
                    print(f"✓ Deep scanned {lib_name} from {lbr_file}")
                except Exception as e:
                    print(f"Error deep scanning {lbr_file.name}: {e}")
            else:
                print(f"✗ Could not find library file for {lib_name} in {self.eagle_dir}")

        return len(used_libraries)

    def _parse_library_quick(self, lbr_path: Path):
        """Quick parse: only extract component names and basic info for matching"""
        try:
            tree = ET.parse(lbr_path)
            root = tree.getroot()
            library_name = lbr_path.stem

            # Find devicesets
            devicesets = root.findall(".//deviceset")
            for deviceset in devicesets:
                device_name = deviceset.get('name', '')
                prefix = deviceset.get('prefix', '')
                description = deviceset.find('description')
                desc_text = description.text if description is not None else ''

                # Find devices (packages)
                devices = deviceset.findall(".//device")
                for device in devices:
                    package = device.get('package', '')
                    device_variant = device.get('name', '')

                    # Create unique key
                    full_name = f"{device_name}{device_variant}"
                    key = self._normalize_component_name(full_name)

                    # Store minimal info (no pin data yet)
                    entry = {
                        'library': library_name,
                        'deviceset': device_name,
                        'device': device_variant,
                        'package': package,
                        'prefix': prefix,
                        'description': desc_text,
                        'pins': {},  # Empty for now
                        'deviceset_elem': None,
                        'device_elem': None,
                        'gate_names': [],
                        'technologies': [],
                        'technology_details': []
                    }
                    self._index_entry_fields(entry)
                    self._store_device_entry(key, entry)

                    # Also store alias with prefix for searching
                    if prefix:
                        prefix_key = f"{prefix}_{key}"
                        self._store_device_entry(prefix_key, entry)
        except Exception as e:
            print(f"Error in quick parse of {lbr_path.name}: {e}")
    
    def _parse_library_deep(self, lbr_path: Path):
        """Deep parse: extract pin coordinates and cache XML for this library"""
        try:
            tree = ET.parse(lbr_path)
            root = tree.getroot()
            library_name = lbr_path.stem
            
            # Cache the full library XML for later embedding
            library_elem = root.find('.//library')
            if library_elem is not None:
                library_elem.set('name', library_name)
                self.library_xml_cache[library_name] = library_elem
            
            # Find devicesets and update pin data for components from this library
            devicesets = root.findall(".//deviceset")
            for deviceset in devicesets:
                device_name = deviceset.get('name', '')
                
                devices = deviceset.findall(".//device")
                for device in devices:
                    device_variant = device.get('name', '')
                    full_name = f"{device_name}{device_variant}"
                    key = self._normalize_component_name(full_name)
                    
                    # Skip if not in device_map (wasn't found during quick scan)
                    if key not in self.device_map:
                        continue
                    
                    technology_names: List[str] = []
                    technology_details: List[Dict[str, List[Dict[str, str]]]] = []
                    technologies_elem = device.find('technologies')
                    if technologies_elem is not None:
                        for tech_elem in technologies_elem.findall('technology'):
                            tech_name = (tech_elem.get('name') or '')
                            technology_names.append(tech_name)
                            attr_details: List[Dict[str, str]] = []
                            for attr_elem in tech_elem.findall('attribute'):
                                attr_details.append({
                                    'name': attr_elem.get('name', ''),
                                    'value': attr_elem.get('value', '')
                                })
                            technology_details.append({
                                'name': tech_name,
                                'attributes': attr_details
                            })

                    # Get pin mappings
                    connects = device.findall(".//connect")
                    pin_map = {}
                    gate_map = {}  # Maps connect.gate to deviceset.gate.name

                    # Detect multi-gate devices where different gates share the
                    # same pin name (e.g. JST connectors: every gate has pin 'S').
                    # In that case, use the gate name as the pin_map key so each
                    # physical pin gets its own entry.
                    pin_name_counts = {}
                    for connect in connects:
                        pn = connect.get('pin', '')
                        pin_name_counts[pn] = pin_name_counts.get(pn, 0) + 1
                    has_dup_pin_names = any(c > 1 for c in pin_name_counts.values())

                    for connect in connects:
                        gate = connect.get('gate', '')
                        pin = connect.get('pin', '')
                        pad = connect.get('pad', '')
                        if has_dup_pin_names:
                            pin_map[gate] = {'gate': gate, 'pad': pad, 'pin_name': pin}
                        else:
                            pin_map[pin] = {'gate': gate, 'pad': pad}
                    
                    # Get pin coordinates from symbols
                    gates = deviceset.findall(".//gate")
                    for gate in gates:
                        gate_name = gate.get('name', 'G$1')
                        symbol_name = gate.get('symbol', '')
                        gate_x_offset = float(gate.get('x', 0))
                        gate_y_offset = float(gate.get('y', 0))
                        
                        # Build mapping from connect.gate to actual gate name
                        for connect in connects:
                            connect_gate = connect.get('gate', '')
                            if connect_gate and connect_gate not in gate_map:
                                gate_map[connect_gate] = gate_name
                        
                        # Find symbol definition
                        symbol = root.find(f".//symbol[@name='{symbol_name}']")
                        if symbol is not None:
                            symbol_pins = symbol.findall('.//pin')
                            for sym_pin in symbol_pins:
                                pin_name = sym_pin.get('name', '')
                                pin_x = float(sym_pin.get('x', 0))
                                pin_y = float(sym_pin.get('y', 0))
                                direction = sym_pin.get('direction') or ''
                                rotation = sym_pin.get('rot') or ''

                                if has_dup_pin_names:
                                    # Match by gate name; store gate offset for
                                    # correct absolute position calculation
                                    if gate_name in pin_map and pin_map[gate_name].get('pin_name') == pin_name:
                                        pin_map[gate_name]['x'] = pin_x
                                        pin_map[gate_name]['y'] = pin_y
                                        pin_map[gate_name]['gate_x'] = gate_x_offset
                                        pin_map[gate_name]['gate_y'] = gate_y_offset
                                        if direction:
                                            pin_map[gate_name]['direction'] = direction
                                        if rotation:
                                            pin_map[gate_name]['rotation'] = rotation
                                elif pin_name in pin_map:
                                    pin_map[pin_name]['x'] = pin_x
                                    pin_map[pin_name]['y'] = pin_y
                                    if direction:
                                        pin_map[pin_name]['direction'] = direction
                                    if rotation:
                                        pin_map[pin_name]['rotation'] = rotation
                                    pin_map[pin_name]['gate'] = gate_name
                    
                    entries = self.device_map.get(key, [])
                    for entry in entries:
                        if (entry['library'] == library_name and
                                entry['deviceset'] == device_name and
                                entry['device'] == device_variant):
                            entry['pins'] = pin_map
                            entry['deviceset_elem'] = deviceset
                            entry['device_elem'] = device
                            entry['gate_names'] = [gate.get('name', 'G$1') for gate in gates]
                            entry['technologies'] = list(technology_names)
                            entry['technology_details'] = list(technology_details)
                            entry['full_path'] = str(lbr_path)
                    
        except Exception as e:
            print(f"Error in deep parse of {lbr_path.name}: {e}")
    
    def _parse_library(self, lbr_path: Path):
        """Parse a single Eagle library file"""
        try:
            tree = ET.parse(lbr_path)
            root = tree.getroot()
            
            library_name = lbr_path.stem
            
            # Cache the full library XML for later embedding
            library_elem = root.find('.//library')
            if library_elem is not None:
                # Ensure name attribute matches the filename
                library_elem.set('name', library_name)
                self.library_xml_cache[library_name] = library_elem
            
            # Find devicesets
            devicesets = root.findall(".//deviceset")
            for deviceset in devicesets:
                device_name = deviceset.get('name', '')
                prefix = deviceset.get('prefix', '')
                description = deviceset.find('description')
                desc_text = description.text if description is not None else ''
                
                # Find devices (packages)
                devices = deviceset.findall(".//device")
                for device in devices:
                    package = device.get('package', '')
                    device_variant = device.get('name', '')
                    
                    # Get connects (pin mappings)
                    connects = device.findall(".//connect")
                    pin_map = {}
                    for connect in connects:
                        gate = connect.get('gate', '')
                        pin = connect.get('pin', '')
                        pad = connect.get('pad', '')
                        pin_map[pin] = {'gate': gate, 'pad': pad}
                    
                    # Get pin coordinates from symbols
                    gates = deviceset.findall(".//gate")
                    for gate in gates:
                        gate_name = gate.get('name', 'G$1')
                        symbol_name = gate.get('symbol', '')
                        gate_x = float(gate.get('x', 0))
                        gate_y = float(gate.get('y', 0))
                        
                        # Find the symbol definition
                        symbol = root.find(f".//symbol[@name='{symbol_name}']")
                        if symbol is not None:
                            # Extract pin positions from symbol
                            symbol_pins = symbol.findall('.//pin')
                            for sym_pin in symbol_pins:
                                pin_name = sym_pin.get('name', '')
                                pin_x = float(sym_pin.get('x', 0))
                                pin_y = float(sym_pin.get('y', 0))
                                direction = sym_pin.get('direction') or ''
                                rotation = sym_pin.get('rot') or ''
                                
                                # Store absolute pin position (gate offset + pin offset)
                                if pin_name in pin_map:
                                    pin_map[pin_name]['x'] = gate_x + pin_x
                                    pin_map[pin_name]['y'] = gate_y + pin_y
                                    if direction:
                                        pin_map[pin_name]['direction'] = direction
                                    if rotation:
                                        pin_map[pin_name]['rotation'] = rotation
                    
                    # Create unique key
                    full_name = f"{device_name}{device_variant}"
                    key = self._normalize_component_name(full_name)
                    
                    entry = {
                        'library': library_name,
                        'deviceset': device_name,
                        'device': device_variant,
                        'package': package,
                        'prefix': prefix,
                        'description': desc_text,
                        'pins': pin_map,
                        'full_path': str(lbr_path),
                        'deviceset_elem': deviceset,
                        'device_elem': device,
                        'gate_names': [gate.get('name', 'G$1') for gate in gates]
                    }
                    self._index_entry_fields(entry)
                    self._store_device_entry(key, entry)

                    if prefix:
                        prefix_key = f"{prefix}_{key}"
                        self._store_device_entry(prefix_key, entry)
        
        except Exception as e:
            print(f"Error parsing library {lbr_path}: {e}")
    
    def _normalize_component_name(self, name: str) -> str:
        """Normalize component name for matching"""
        return re.sub(r'[^a-zA-Z0-9]', '', name.upper())
    
    def _select_entry(self, entries: List[Dict], desired_prefix: Optional[str] = None,
                       package_hint: str = '') -> Optional[Dict]:
        """Choose a representative entry from a list, preferring matching prefixes
        and package type when specified."""
        if not entries:
            return None

        pkg_hint = package_hint.upper()
        want_axial = any(k in pkg_hint for k in ('AXIAL', 'THT', 'THROUGH'))
        want_radial = any(k in pkg_hint for k in ('RAD', 'RADIAL'))
        want_electrolytic = 'ELECTROLYTIC' in pkg_hint
        want_dip = 'DIP' in pkg_hint
        want_to_pkg = bool(re.search(r'\bTO[-\s]?\d', pkg_hint))
        want_through_hole = want_axial or want_radial or want_electrolytic or want_dip or want_to_pkg

        # Extract a pin count from package hint like "DIP-14" or "DIP-16"
        dip_pin_count = 0
        if want_dip:
            dip_match = re.search(r'DIP[-\s]?(\d+)', pkg_hint)
            if dip_match:
                dip_pin_count = int(dip_match.group(1))

        # Extract TO-package number (e.g. "TO-220" → "220")
        to_pkg_num = ''
        if want_to_pkg:
            to_match = re.search(r'TO[-\s]?(\d+)', pkg_hint)
            if to_match:
                to_pkg_num = to_match.group(1)

        # SMD size codes from the BOM package field (e.g. "0805", "0603")
        smd_size = ''
        for sz in ('0201', '0402', '0603', '0805', '1206', '1210', '2010', '2512'):
            if sz in pkg_hint:
                smd_size = sz
                break

        # Build a prioritized list filtered by prefix
        candidates = []
        if desired_prefix:
            for entry in entries:
                if self._prefix_matches(desired_prefix, entry.get('prefix')):
                    candidates.append(entry)
        if not candidates:
            candidates = list(entries)

        if not candidates:
            return None

        if want_through_hole or smd_size:
            smd_markers = ('0805', '0603', '1206', '0402', '0201', 'SMD')

            def _package_score(entry: Dict) -> int:
                pkg = (entry.get('package') or '').upper()
                dev = (entry.get('device') or '').upper()
                ds = (entry.get('deviceset') or '').upper()
                combined = pkg + ' ' + dev + ' ' + ds
                is_smd = any(s in combined for s in smd_markers)
                has_slash = '/' in dev
                is_polarized = 'CPOL' in ds or 'POL' in combined
                is_axial_pkg = 'AXIAL' in combined or (has_slash and not is_smd)

                if want_to_pkg:
                    has_to = bool(re.search(r'TO[-_]?\d', combined))
                    if has_to:
                        if to_pkg_num and to_pkg_num in combined:
                            return 0
                        return 0
                    if is_smd:
                        return 3
                    return 2
                if want_dip:
                    is_dip = 'DIL' in combined or 'DIP' in combined or 'PDIP' in combined
                    # NXP/TI convention: deviceset ending with digit+N = DIP
                    if not is_dip:
                        ds_suffix_match = re.match(r'^.*\d([A-Z]{1,3})$', ds)
                        if ds_suffix_match and ds_suffix_match.group(1) in ('N', 'P', 'AN'):
                            is_dip = True
                    if is_dip:
                        if dip_pin_count and str(dip_pin_count) in combined:
                            return 0
                        return 0
                    # Explicit SMD IC package markers
                    smd_ic_markers = ('SOIC', 'SOP', 'SSOP', 'TSSOP', 'MSOP',
                                      'QFP', 'LQFP', 'TQFP', 'QFN', 'DFN',
                                      'LFCSP', 'WSON', 'BGA', 'LGA', 'CSP',
                                      'PLCC', 'SC70')
                    if any(m in combined for m in smd_ic_markers):
                        return 3
                    # NXP/TI convention: deviceset ending with D/DB/PW = SMD
                    ds_suffix_match2 = re.match(r'^.*\d([A-Z]{1,3})$', ds)
                    if ds_suffix_match2:
                        smd_suffix = ds_suffix_match2.group(1)
                        if smd_suffix in ('D', 'DB', 'PW', 'DBV', 'RGY', 'RGT', 'DGK'):
                            return 3
                    # SOT packages: SOT23/89/223/363 are SMD; SOT38/27 are NXP DIP codes
                    sot_match = re.search(r'SOT(\d+)', combined)
                    if sot_match:
                        sot_num = int(sot_match.group(1))
                        if sot_num in (23, 89, 143, 223, 323, 353, 363, 457, 666):
                            return 3
                    return 2
                if want_axial and want_electrolytic:
                    if is_polarized and is_axial_pkg:
                        return 0
                    if is_polarized and not is_smd:
                        return 1
                    if is_axial_pkg:
                        return 1
                    return 3 if is_smd else 2
                if want_axial:
                    if is_axial_pkg:
                        return 0
                    return 3 if is_smd else 1
                if want_electrolytic:
                    if is_polarized and not is_smd:
                        return 0
                    if is_polarized:
                        return 1
                    return 3 if is_smd else 2
                if want_radial:
                    if 'RAD' in combined and not is_smd:
                        return 0
                    if has_slash and not is_smd:
                        return 0
                    return 3 if is_smd else 1
                if smd_size:
                    if smd_size in combined:
                        return 0
                    return 1
                return 1

            candidates.sort(key=_package_score)

        return candidates[0]

    def _prefix_matches(self, desired: Optional[str], actual: Optional[str]) -> bool:
        if not desired:
            return True
        desired_upper = desired.upper()
        actual_upper = (actual or '').upper()
        if desired_upper == actual_upper:
            return True
        if not actual_upper:
            return True

        connector_prefixes = {'J', 'X', 'SV', 'P', 'CON', 'CN'}
        equivalents = [
            connector_prefixes,
            {'TP', 'T'},
            {'LED', 'D'},
            {'Q', 'T'},
        ]

        for group in equivalents:
            if desired_upper in group and actual_upper in group:
                return True
        return False

    def _find_package_variant(self, base_entry: Dict, package_hint: str) -> Optional[Dict]:
        """Given a matched component, find a variant in the same library/deviceset
        that better matches the requested package (e.g. DIP vs SOIC)."""
        if not base_entry or not package_hint:
            return None
        lib = base_entry.get('library', '')
        ds = base_entry.get('deviceset', '')
        if not lib or not ds:
            return None

        # Collect all variants of the same deviceset
        siblings = []
        for entry in self._iter_unique_devices():
            if entry.get('library') == lib and entry.get('deviceset') == ds:
                siblings.append(entry)

        if len(siblings) <= 1:
            return None

        best = self._select_entry(siblings, base_entry.get('prefix'), package_hint)
        if best and best is not base_entry:
            return best
        return None

    def _entry_matches_token(self, entry: Dict, norm_key: str) -> bool:
        """Return True if the normalized token appears within key fields."""
        if not norm_key:
            return False

        fields = (
            entry.get('deviceset', ''),
            entry.get('device', ''),
            entry.get('package', ''),
            entry.get('description', '')
        )

        for field in fields:
            if not field:
                continue
            field_norm = self._normalize_component_name(field)
            if not field_norm:
                continue
            if (len(norm_key) >= 3 and len(field_norm) >= 3 and
                    (norm_key in field_norm or field_norm in norm_key)):
                return True
            if norm_key == field_norm and norm_key:
                return True
        return False

    def _find_by_tokens(self, tokens: List[str], prefix: Optional[str]) -> Optional[Dict]:
        if not tokens:
            return None
        normalized = [self._normalize_component_name(token) for token in tokens if token]
        normalized = [token for token in normalized if token]
        if not normalized:
            return None

        for token in normalized:
            indexed = self.part_index.get(token)
            if indexed:
                entry = self._select_entry(indexed, prefix)
                if entry and self._prefix_matches(prefix, entry.get('prefix')):
                    return entry
            for entry in self._iter_unique_devices():
                if not self._prefix_matches(prefix, entry.get('prefix')):
                    continue
                if self._entry_matches_token(entry, token):
                    return entry
        return None

    def _find_all_token_matches(self, tokens: List[str], prefix: Optional[str],
                                limit: int = 50) -> List[Dict]:
        """Find ALL entries matching the given tokens, not just the first."""
        if not tokens:
            return []
        normalized = [self._normalize_component_name(t) for t in tokens if t]
        normalized = [t for t in normalized if t]
        if not normalized:
            return []

        results = []
        seen = set()
        for token in normalized:
            indexed = self.part_index.get(token)
            if indexed:
                for entry in indexed:
                    eid = id(entry)
                    if eid not in seen and self._prefix_matches(prefix, entry.get('prefix')):
                        seen.add(eid)
                        results.append(entry)
            for entry in self._iter_unique_devices():
                eid = id(entry)
                if eid in seen:
                    continue
                if not self._prefix_matches(prefix, entry.get('prefix')):
                    continue
                if self._entry_matches_token(entry, token):
                    seen.add(eid)
                    results.append(entry)
                    if len(results) >= limit:
                        return results
        return results

    def _extract_part_tokens(self, component_value: str) -> List[str]:
        tokens: List[str] = []
        if not component_value:
            return tokens

        value_upper = component_value.upper()
        if value_upper in {'GENERIC', 'NONE', 'N/A', 'NA'}:
            return tokens

        compact = re.sub(r'[^A-Z0-9]', '', value_upper)
        if len(compact) >= 6:
            tokens.append(compact)

        raw_tokens = re.findall(r'[A-Z0-9]{4,}', value_upper)
        for token in raw_tokens:
            if token in tokens:
                continue
            if token.isdigit() and len(token) < 5:
                continue
            tokens.append(token)

        return tokens

    def find_component(self, component_ref: str, component_value: str = "",
                       component_package: str = "") -> Optional[Dict]:
        """Find best matching component in libraries"""
        # Check manual mappings first
        if component_ref in self.manual_mappings:
            mapping = self.manual_mappings[component_ref]
            # Find the full device info
            for device in self._iter_unique_devices():
                if (device['library'] == mapping['library'] and 
                    device['deviceset'] == mapping['deviceset'] and
                    device['device'] == mapping.get('device', '')):
                    return device
        
        # Extract prefix (R, C, U, etc.) and number
        match = re.match(r'^([A-Z]+)(\d+)', component_ref)
        if not match:
            return None
        
        prefix = match.group(1)
        value_upper = component_value.upper()
        pkg_upper = component_package.upper()

        # Determine if the BOM requests through-hole packaging
        is_electrolytic = 'ELECTROLYTIC' in pkg_upper or 'ELECTROLYTIC' in value_upper
        is_axial = any(k in pkg_upper for k in ('AXIAL', 'THT', 'THROUGH'))
        is_radial = any(k in pkg_upper for k in ('RAD', 'RADIAL'))
        is_to_package = bool(re.search(r'\bTO[-\s]?\d', pkg_upper))
        is_through_hole = is_axial or is_radial or is_electrolytic or 'DIP' in pkg_upper or is_to_package
        
        # Try to match based on value and prefix
        search_keys = []

        pn_tokens = self._extract_part_tokens(component_value)

        # For DIP ICs, prepend tokens with "N" suffix (NXP/TI convention for DIP).
        # This ensures 74HC74N (DIP) is found before 74HC74D (SOP).
        want_dip = 'DIP' in pkg_upper
        if want_dip and pn_tokens and prefix in ('U', 'IC'):
            dip_tokens = [t + 'N' for t in pn_tokens if not t.upper().endswith('N')]
            pn_tokens = dip_tokens + pn_tokens

        # For passives (R/C/L) with through-hole hints, skip the token shortcut
        # because it doesn't consider package and would pick SMD variants.
        # For ICs and other actives, always run it to find the correct part,
        # then try to find a better package variant of the same deviceset.
        if is_through_hole and prefix in ('R', 'C', 'L'):
            pass  # skip token search for passives — use search_keys path instead
        elif pn_tokens:
            pn_match = self._find_by_tokens(pn_tokens, prefix)
            if pn_match and is_through_hole:
                # Found the right IC but might be wrong package variant.
                # First try within the same deviceset.
                better = self._find_package_variant(pn_match, component_package)
                if better:
                    return better
                # Same deviceset had no variant — search ALL matching entries
                # across different devicesets (e.g. NXP 74HC74D vs 74HC74N).
                original_tokens = self._extract_part_tokens(component_value)
                all_matches = self._find_all_token_matches(original_tokens, prefix)
                if len(all_matches) > 1:
                    best = self._select_entry(all_matches, prefix, component_package)
                    if best:
                        return best
                return pn_match
            elif pn_match:
                return pn_match
        
        # For resistors
        if prefix == 'R':
            search_keys = ['R-EU_', 'R-US_', 'RESISTOR']
            if is_axial or is_through_hole:
                search_keys = ['R-EU_0207/10', 'R-EU_0204/7', 'R-EU_0207', 'R-EU_0309',
                               'R-EU_0411', 'R-EU_0204'] + search_keys
            elif '0805' in value_upper or '0805' in pkg_upper:
                search_keys = ['R-EU_R0805', 'R0805', 'R-US_R0805'] + search_keys
            elif '0603' in value_upper or '0603' in pkg_upper:
                search_keys = ['R-EU_R0603', 'R0603', 'R-US_R0603'] + search_keys
            elif '1206' in value_upper or '1206' in pkg_upper:
                search_keys = ['R-EU_R1206', 'R1206', 'R-US_R1206'] + search_keys
            else:
                search_keys = ['R-EU_R0805', 'R0805'] + search_keys
        
        # For capacitors
        elif prefix == 'C':
            search_keys = ['C-EU', 'C-US', 'CPOL-EU', 'CAPACITOR']
            if is_axial and is_electrolytic:
                search_keys = ['CPOL-EUE', 'CPOL-EU', 'CPOL-US'] + search_keys
            elif is_through_hole and is_electrolytic:
                search_keys = ['CPOL-EUE2.5-5', 'CPOL-EUE2-5', 'CPOL-EUE3.5-8',
                               'CPOL-EU', 'CPOL-US'] + search_keys
            elif is_through_hole:
                search_keys = ['C-EU050-024X044', 'C-EU025-024X044', 'C-EU050',
                               'C-EU025', 'C-EU'] + search_keys
            elif '0805' in value_upper or '0805' in pkg_upper:
                search_keys = ['C-EUC0805', 'C0805', 'C-USC0805'] + search_keys
            elif '0603' in value_upper or '0603' in pkg_upper:
                search_keys = ['C-EUC0603', 'C0603', 'C-USC0603'] + search_keys
            elif '1206' in value_upper or '1206' in pkg_upper:
                search_keys = ['C-EUC1206', 'C1206', 'C-USC1206'] + search_keys
            elif is_electrolytic:
                search_keys = ['CPOL-EU', 'CPOL-US', 'C-EU'] + search_keys
            else:
                search_keys = ['C-EUC0805', 'C0805'] + search_keys
        
        # For inductors
        elif prefix == 'L':
            search_keys = ['L-EU', 'L-US', 'INDUCTOR', 'FERRITE', 'L']
            value_tokens = re.findall(r'[A-Z0-9]{3,}', value_upper)
            prioritized_tokens = []
            for token in value_tokens:
                if any(ch.isdigit() for ch in token) and any(ch.isalpha() for ch in token):
                    prioritized_tokens.append(token)
            if prioritized_tokens:
                search_keys = prioritized_tokens + search_keys
        
        # For ICs and integrated circuits
        elif prefix == 'U' or prefix == 'IC':
            # Try to extract IC part number from value
            ic_patterns = [
                r'(AD\d+)',      # Analog Devices
                r'(PCM\d+)',     # PCM ICs
                r'(LM\d+)',      # LM series
                r'(TL\d+)',      # TL series
                r'(OP\d+)',      # Op-amps
                r'(AT\d+)',      # Atmel
                r'(\d+[A-Z]\d+)', # Generic pattern
            ]
            
            for pattern in ic_patterns:
                ic_match = re.search(pattern, value_upper)
                if ic_match:
                    ic_part = ic_match.group(1)
                    search_keys.append(ic_part)
                    break
            
            # Common IC types
            if 'OPAMP' in value_upper or 'OP-AMP' in value_upper:
                search_keys.extend(['OPAMP', 'OP-AMP'])
            elif 'EEPROM' in value_upper:
                search_keys.extend(['24', '25', 'EEPROM'])
            
            search_keys.extend(['IC', 'U'])
        
        # For diodes
        elif prefix == 'D':
            if 'OR-ING' in value_upper or 'ORING' in value_upper or 'SCHOTTKY' in value_upper:
                search_keys = ['SCHOTTKY', 'DIODE-SCHOTTKY', 'D-SCHOTTKY']
            else:
                search_keys = ['DIODE', 'D']
        
        # For LEDs
        elif prefix == 'LED':
            if 'AMBER' in value_upper or 'YELLOW' in value_upper:
                search_keys = ['LED', 'LEDYELLOW', 'LEDAMBER']
            elif 'GREEN' in value_upper:
                search_keys = ['LED', 'LEDGREEN']
            elif 'RED' in value_upper:
                search_keys = ['LED', 'LEDRED']
            else:
                search_keys = ['LED', 'LED3MM', 'LED5MM']
        
        # For connectors
        elif prefix == 'J':
            search_keys = []
            # Determine pin count from value or connections
            pin_count = 0

            # Try to extract pin count from value
            pin_match = re.search(r'(\d+)[-\s]?(?:PIN|pin)', value_upper)
            if pin_match:
                pin_count = int(pin_match.group(1))

            # Prioritize explicit part numbers (e.g., 32005-601, 104087-0801)
            compact_value = re.sub(r'[^A-Z0-9]', '', value_upper)
            identifier_tokens: List[str] = []
            if len(compact_value) >= 6:
                identifier_tokens.append(compact_value)

            raw_tokens = re.findall(r'[A-Z0-9]{4,}', value_upper)
            for token in raw_tokens:
                if token not in identifier_tokens:
                    identifier_tokens.append(token)

            if identifier_tokens:
                search_keys.extend(identifier_tokens)

            # Check for specific connector types
            if 'MOLEX' in value_upper:
                # Molex connectors - try to find exact part number
                molex_part = re.search(r'(104087[-\d]+)', value_upper)
                if molex_part:
                    part_token = re.sub(r'[^A-Z0-9]', '', molex_part.group(1).upper())
                    if part_token not in search_keys:
                        search_keys.insert(0, part_token)
                else:
                    search_keys.append('MOLEX')
                    
                # Molex 104087-0801 is 8-pin
                if '104087' in value_upper or '0801' in value_upper:
                    pin_count = 8
                    search_keys.extend(['PINHD-1X8', 'MA08', 'HEADER-1X8'])
                    
            elif 'RCA' in value_upper:
                search_keys.extend(['RCA', 'CINCH', 'PHONOJACK'])
                pin_count = 2  # RCA has 2 pins (signal + ground)
            elif 'RCJ' in value_upper:
                search_keys.extend(['RCJ', 'RCA', 'PHONOJACK', 'CUI'])
                pin_count = max(pin_count, 2)
                
            elif 'BARREL' in value_upper or 'PJ-002' in value_upper or 'DC' in value_upper:
                search_keys.extend(['DC-JACK', 'POWER-JACK', 'BARREL'])
                pin_count = 3  # Barrel jack typically has 3 pins (tip, ring, sleeve)
                
            elif 'HEADER' in value_upper or 'PIN' in value_upper:
                if pin_count > 0:
                    search_keys.extend([f'PINHD-1X{pin_count}', f'HEADER-1X{pin_count}', f'MA{pin_count:02d}'])
                else:
                    search_keys.extend(['PINHD', 'HEADER'])
            else:
                # Generic connector - try to determine pin count
                if pin_count > 0:
                    search_keys.extend([f'PINHD-1X{pin_count}', f'MA{pin_count:02d}'])
                else:
                    search_keys.extend(['CON', 'CONN', 'HEADER', 'PINHD'])
            
            # NEVER use BANANA_CONN for real connectors
            # Filter it out from results later
        
        # For transistors/MOSFETs
        elif prefix == 'Q':
            search_keys = ['MOSFET', 'TRANSISTOR', 'BJT', 'FET']
            # Extract part number from value (e.g. IRLZ44N, 2N2222, BC547)
            part_tokens = re.findall(r'[A-Z0-9]{3,}', value_upper)
            for token in part_tokens:
                if token not in search_keys and not token.isdigit():
                    search_keys.insert(0, token)
            if 'TO-220' in value_upper or 'TO220' in value_upper:
                search_keys.extend(['TO220', 'TO-220'])
            search_keys.append('Q')

        # For crystals/oscillators
        elif prefix == 'Y':
            search_keys = ['CRYSTAL', 'XTAL', 'RESONATOR', 'Y']
        
        # For Arduino pins or test points
        elif prefix == 'A':
            search_keys = ['TP', 'TESTPOINT', 'PAD']
        
        # Generic search
        else:
            search_keys = [prefix]
        
        # Search for matching component with partial matching
        for key in search_keys:
            norm_key = self._normalize_component_name(key)
            
            # Exact match
            if norm_key in self.device_map:
                entry = self._select_entry(self.device_map[norm_key], prefix, component_package)
                if entry:
                    if prefix == 'J':
                        name_upper = entry['deviceset'].upper()
                        if 'BANANA' in name_upper:
                            continue
                        if 'TESTPOINT' in name_upper or 'TP' in name_upper:
                            continue
                    return entry
            
            # Try with prefix
            prefix_key = f"{prefix}_{norm_key}"
            if prefix_key in self.device_map:
                entry = self._select_entry(self.device_map[prefix_key], prefix, component_package)
                if entry:
                    if prefix == 'J':
                        name_upper = entry['deviceset'].upper()
                        if 'BANANA' in name_upper or 'TESTPOINT' in name_upper:
                            continue
                    return entry
            
            # Partial match - collect candidates and rank by package fit
            seen_ids = set()
            partial_candidates = []
            for dev_key, entries in self.device_map.items():
                # Both strings must be >= 3 chars for substring matching.
                # Prevents short keys like 'L' matching inside 'MOSFET',
                # and short search terms like 'Q' matching 'TPS62153AQRGTRQ1'.
                if len(norm_key) >= 3 and len(dev_key) >= 3:
                    matches_key = norm_key in dev_key or dev_key in norm_key
                elif len(norm_key) >= 3:
                    matches_key = norm_key == dev_key
                else:
                    matches_key = norm_key == dev_key
                for entry in entries:
                    if not matches_key and not self._entry_matches_token(entry, norm_key):
                        continue
                    entry_id = id(entry)
                    if entry_id in seen_ids:
                        continue
                    seen_ids.add(entry_id)
                    if self._prefix_matches(prefix, entry.get('prefix')):
                        if prefix == 'J':
                            name_upper = entry['deviceset'].upper()
                            if 'BANANA' in name_upper or 'TESTPOINT' in name_upper:
                                continue
                        partial_candidates.append(entry)
            if partial_candidates:
                return self._select_entry(partial_candidates, prefix, component_package)
        
        # Fallback: search for any component with matching prefix.
        # Use strict matching here — require actual prefix to be non-empty
        # to avoid random entries (e.g. adafruit/MAX1898 with no prefix)
        # matching searches for U, Q, etc.
        fallback_candidates = []
        for device in self._iter_unique_devices():
            actual_prefix = (device.get('prefix') or '').strip()
            if actual_prefix and self._prefix_matches(prefix, actual_prefix):
                fallback_candidates.append(device)
                if len(fallback_candidates) >= 50:
                    break
        if fallback_candidates:
            return self._select_entry(fallback_candidates, prefix, component_package)
        
        return None


class MDSchematicParser:
    """Parse markdown schematic files"""
    
    def __init__(self, md_file: str):
        self.md_file = Path(md_file)
        self.components = []
        self.nets = []
        self.connections = []
        self.net_nodes: Dict[str, Dict] = {}
        self.net_name_lookup: Set[str] = set()
        self.pin_label_map: Dict[str, Dict[str, str]] = {}
        
    def parse(self):
        """Parse the markdown file"""
        # Reset parsed data each time parse() runs
        self.components = []
        self.nets = []
        self.connections = []
        self.net_nodes = {}
        self.net_name_lookup = set()
        self.pin_label_map = {}

        try:
            try:
                with open(self.md_file, 'r', encoding='utf-8') as f:
                    content = f.read()
            except UnicodeDecodeError:
                # Fallback for files with Latin-1/Windows-1252 chars (e.g. µ, Ω)
                with open(self.md_file, 'r', encoding='cp1252') as f:
                    content = f.read()

            self._parse_components(content)
            self._parse_nets(content)
            self._parse_connections(content)
            self.warnings = self._validate_cross_references()
            self._validate_supply_net_isolation()

            self.data = {
                'components': self.components,
                'nets': self.nets,
                'connections': self.connections,
                'warnings': self.warnings
            }
            return self.data
        except FileNotFoundError:
            raise Exception(f"Markdown file not found: {self.md_file}")
        except re.error as e:
            raise Exception(f"Regex error in parsing: {e}. Please check MD file format.")
        except Exception as e:
            raise Exception(f"Error parsing MD file: {str(e)}")

    def _validate_cross_references(self) -> List[str]:
        """Detect pin-number disagreements between component sections.

        The dangerous pattern: component A's section declares A.PinX→B.PinY,
        but component B's section declares B.PinZ→A.PinX where Y≠Z.  Both
        sections reference the same wire (A.PinX) but disagree on which B
        pin it connects to.  This off-by-one silently cascades into a bus.

        Normal fan-out (one signal to multiple IC inputs) is NOT flagged
        because both sections agree on every individual pin-to-pin link.
        Same-net fan-in (multiple pins on one IC all tied to GND) is also
        excluded — the pins legitimately share a net even though each
        section only lists one target."""
        warnings: List[str] = []
        known_refs = {c['reference'] for c in self.components}

        # Net membership lookup: (ref, pin) → set of canonical net names
        pin_nets: Dict[Tuple[str, str], Set[str]] = {}
        for net_key, entry in self.net_nodes.items():
            for node in entry.get('nodes', []):
                pin_nets.setdefault((node['ref'], node['pin']), set()).add(net_key)

        # Build directed connection map using only authoritative connections
        # (skip pattern2 intermediates and net-summary anchor connections)
        # forward[(A,B)][pinX] = set of B pins that A.PinX claims to connect to
        forward: Dict[Tuple[str, str], Dict[str, Set[str]]] = {}
        for conn in self.connections:
            if conn.get('intermediate'):
                continue
            f_ref = conn['from']['ref']
            t_ref = conn['to']['ref']
            if f_ref not in known_refs or t_ref not in known_refs:
                continue
            if f_ref == t_ref:
                continue
            f_pin = conn['from']['pin']
            t_pin = conn['to']['pin']
            forward.setdefault((f_ref, t_ref), {}).setdefault(f_pin, set()).add(t_pin)

        # For each reverse connection B.PinZ→A.PinW, check that the
        # forward direction A.PinW→B.PinY includes Y==Z.
        seen = set()
        for (b_ref, a_ref), b_pins in forward.items():
            a_to_b = forward.get((a_ref, b_ref))
            if not a_to_b:
                continue
            for b_pin_z, a_pin_set in b_pins.items():
                for a_pin_w in a_pin_set:
                    fwd_b_pins = a_to_b.get(a_pin_w)
                    if not fwd_b_pins:
                        continue
                    if b_pin_z not in fwd_b_pins:
                        # Suppress if b_pin_z and all fwd_b_pins share a net
                        # (legitimate fan-in on a shared net like GND/ADJ_Q)
                        z_nets = pin_nets.get((b_ref, b_pin_z), set())
                        if z_nets and all(
                            z_nets & pin_nets.get((b_ref, p), set())
                            for p in fwd_b_pins
                        ):
                            continue

                        key = (min(a_ref, b_ref), max(a_ref, b_ref), a_pin_w)
                        if key in seen:
                            continue
                        seen.add(key)
                        fwd_list = ', '.join(sorted(fwd_b_pins, key=lambda p: int(p) if p.isdigit() else p))
                        warnings.append(
                            f"CONFLICT: {a_ref}.Pin{a_pin_w} → {b_ref}.Pin{fwd_list} "
                            f"(per {a_ref}'s section), but {b_ref}.Pin{b_pin_z} → "
                            f"{a_ref}.Pin{a_pin_w} (per {b_ref}'s section). "
                            f"Off-by-one error?"
                        )

        return warnings

    def _validate_supply_net_isolation(self) -> None:
        """Ensure no pin appears on multiple supply nets (would short supplies)."""
        pin_to_supply_nets: Dict[Tuple[str, str], Set[str]] = {}
        for net_key, entry in self.net_nodes.items():
            net_name = entry.get('name', net_key)
            if not self._is_supply_net(net_name):
                continue
            for node in entry.get('nodes', []):
                ref, pin = node.get('ref'), node.get('pin')
                if not ref or not pin:
                    continue
                pin_to_supply_nets.setdefault((ref, pin), set()).add(net_name)

        conflicts: List[str] = []
        for (ref, pin), nets in pin_to_supply_nets.items():
            if len(nets) > 1:
                nets_sorted = sorted(nets)
                conflicts.append(
                    f"SUPPLY SHORT: {ref}.Pin{pin} appears on multiple supply nets: {', '.join(nets_sorted)}. "
                    f"Fix the MD so each pin is on only one supply net."
                )
        if conflicts:
            msg = "Supply net isolation violated:\n  " + "\n  ".join(conflicts)
            raise Exception(msg)

    def _parse_components(self, content: str):
        """Parse component list/BOM from markdown."""
        bom_patterns = [
            r'##\s*COMPLETE\s+BILL\s+OF\s+MATERIALS.*?\n(.*?)(?=\n##[^#]|\Z)',
            r'##\s*BILL\s+OF\s+MATERIALS.*?\n(.*?)(?=\n##[^#]|\Z)',
            r'##\s*BOM\s*\(.*?\).*?\n(.*?)(?=\n##[^#]|\Z)',
            r'##\s*BOM\s*[-:].*?\n(.*?)(?=\n##[^#]|\Z)',
            r'##\s*BOM\s*\n(.*?)(?=\n##[^#]|\Z)',
        ]

        bom_section = None
        for pattern in bom_patterns:
            bom_match = re.search(pattern, content, re.IGNORECASE | re.MULTILINE | re.DOTALL)
            if bom_match:
                bom_section = bom_match.group(1)
                break

        if not bom_section:
            raise Exception("No BOM section found. Looking for: '## BOM' or '## BILL OF MATERIALS'")

        table_components = self._parse_table_components(bom_section)
        if table_components:
            self.components = table_components
            return

        code_blocks = re.findall(r'```\s*(.*?)\s*```', bom_section, re.DOTALL)
        if not code_blocks:
            raise Exception("No component lists found in BOM section (tried tables and code blocks)")

        for block in code_blocks:
            for raw_line in block.split('\n'):
                line = raw_line.strip()
                if not line or ':' not in line:
                    continue

                match = re.match(r'^([A-Z]+\d+(?:,\s*[A-Z]+\d+)*)\s*:\s*(.+)$', line)
                if not match:
                    continue

                refs_part = match.group(1)
                value_part = match.group(2)
                refs = [r.strip() for r in refs_part.split(',')]

                for ref in refs:
                    package = ''
                    value = value_part

                    pkg_match = re.search(r'\(([^)]+)\)', value_part)
                    if pkg_match:
                        pkg = pkg_match.group(1)
                        if any(x in pkg.upper() for x in ['0805', '0603', '1206', 'SOIC', 'TSSOP', 'SOT', 'LFCSP', 'SMA', 'DO-', 'SMD', 'DIP', 'ELECTROLYTIC']):
                            package = pkg
                            value = value_part.split('(')[0].strip()

                    if ' - ' in value:
                        value = value.split(' - ')[0].strip()
                    if ' or ' in value:
                        value = value.split(' or ')[0].strip()

                    value = normalize_component_value(value)
                    self.components.append({
                        'reference': ref,
                        'value': value,
                        'package': package if package else self._infer_package(ref, value)
                    })

        if not self.components:
            raise Exception("No components extracted from BOM section")

    def _parse_table_components(self, bom_section: str) -> List[Dict[str, str]]:
        """Parse components from markdown tables."""
        components = []
        lines = bom_section.split('\n')
        in_table = False
        header_indices = {}  # Map column names to indices

        for raw_line in lines:
            line = raw_line.strip()
            if not line.startswith('|'):
                in_table = False
                header_indices = {}
                continue

            # Parse header row to find column positions
            if 'Ref' in line and ('Value' in line or 'Description' in line or 'Part' in line):
                cells = [c.strip().lower() for c in line.split('|') if c.strip()]
                for idx, cell in enumerate(cells):
                    if 'ref' in cell:
                        header_indices['ref'] = idx
                    elif 'qty' in cell:
                        header_indices['qty'] = idx
                    elif 'part' in cell and 'number' in cell:
                        header_indices['part_number'] = idx
                    elif cell == 'value' or cell.startswith('value'):
                        header_indices['value'] = idx
                    elif 'desc' in cell:
                        header_indices['description'] = idx
                    elif 'pack' in cell:
                        header_indices['package'] = idx
                in_table = True
                continue

            if '---' in line:
                in_table = True
                continue

            if not in_table:
                continue

            cells = [c.strip() for c in line.split('|') if c.strip()]
            if len(cells) < 3:
                continue

            ref_idx = header_indices.get('ref', 0)
            ref_cell = cells[ref_idx] if ref_idx < len(cells) else ''
            if not re.search(r'[A-Z]+\d+', ref_cell):
                continue

            refs = re.findall(r'([A-Z]+\d+)', ref_cell)
            
            # Get values from appropriate columns
            part_number = cells[header_indices['part_number']] if 'part_number' in header_indices and header_indices['part_number'] < len(cells) else ''
            value_col = cells[header_indices['value']] if 'value' in header_indices and header_indices['value'] < len(cells) else ''
            desc_col = cells[header_indices['description']] if 'description' in header_indices and header_indices['description'] < len(cells) else ''
            package = cells[header_indices['package']] if 'package' in header_indices and header_indices['package'] < len(cells) else ''
            
            for ref in refs:
                # For passive components (R, C, L), prefer the Value column if it has electrical notation
                prefix = re.match(r'^([A-Z]+)', ref)
                prefix = prefix.group(1) if prefix else ''
                
                if prefix in ('R', 'C', 'L') and value_col:
                    # Check if value_col contains electrical notation (incl. ? as ohm placeholder)
                    if re.search(r'[\dkKmMµunp]?[ΩFH?ohmfaradhenry]|[0-9]+[kKmMµunp]', value_col, re.IGNORECASE):
                        value = value_col
                    else:
                        value = value_col if value_col else (desc_col if desc_col else part_number)
                else:
                    # For other components, use part number or description
                    value = part_number if part_number else (value_col if value_col else desc_col)

                value = normalize_component_value(value)
                components.append({
                    'reference': ref,
                    'value': value,
                    'package': package if package else self._infer_package(ref, value)
                })

        return components

    def _parse_nets(self, content: str):
        """Parse net definitions from markdown."""
        try:
            net_pattern = r'(?:Net|Signal)\s+([A-Z_][A-Z0-9_]*)\b'
            matches = re.findall(net_pattern, content, re.IGNORECASE)
            for net_name in matches:
                self._add_net_name(net_name)
        except re.error:
            pass

        common_nets = ['VCC', 'VDD', 'GND', 'VSS', '3V3', 'VOUT', 'VIN']
        upper_content = content.upper()
        for common_net in common_nets:
            if common_net in upper_content:
                self._add_net_name(common_net)

        self._parse_net_memberships(content)

    def _add_net_name(self, net_name: Optional[str]):
        """Record a net name once, preserving insertion order."""
        if not net_name:
            return
        cleaned = net_name.strip()
        if not cleaned:
            return
        if self._should_ignore_net(cleaned):
            return
        canonical = cleaned.upper()
        if canonical in self.net_name_lookup:
            return
        self.net_name_lookup.add(canonical)
        self.nets.append(cleaned)

    def _parse_net_memberships(self, content: str):
        """Capture pin membership from 'Net NAME:' summary lines."""
        line_pattern = re.compile(r'^(?:Net|Signal)\s+([A-Z_][A-Z0-9_]*)\s*(?:\([^)]+\))?\s*:\s*(.+)$', re.IGNORECASE)
        # Optional (label) after pin number, e.g. U4.Pin3 (1Y) or U4.Pin3
        pin_pattern = re.compile(r'([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?', re.IGNORECASE)
        pad_pattern = re.compile(r'([A-Z]+\d+)\.(?:PAD)', re.IGNORECASE)

        for raw_line in content.split('\n'):
            line = raw_line.strip()
            if not line:
                continue
            match = line_pattern.match(line)
            if not match:
                continue
            net_name = match.group(1)
            members = match.group(2)
            for m in pin_pattern.finditer(members):
                ref, pin, label = m.group(1), m.group(2), m.group(3)
                self._register_net_node(net_name, ref, pin, label)
            for ref in pad_pattern.findall(members):
                self._register_net_node(net_name, ref, 'PAD')

    def _register_net_node(self, net_name: Optional[str], ref: Optional[str], pin: Optional[str], label: Optional[str] = None):
        """Track that ref.pin participates in a named net."""
        if not net_name or not ref or not pin:
            return
        if self._should_ignore_net(net_name):
            return
        cleaned_ref = ref.strip()
        cleaned_pin = pin.strip()
        if not cleaned_ref or not cleaned_pin:
            return
        canonical_net = net_name.strip().upper()
        if not canonical_net:
            return
        entry = self.net_nodes.setdefault(canonical_net, {
            'name': net_name.strip(),
            'nodes': []
        })
        label_val = label.strip() if label else None
        # Avoid duplicate entries; update label if we have one and existing doesn't
        for node in entry['nodes']:
            if node['ref'] == cleaned_ref and node['pin'] == cleaned_pin:
                if label_val and not node.get('label'):
                    node['label'] = label_val
                return
        entry['nodes'].append({
            'ref': cleaned_ref,
            'pin': cleaned_pin,
            'label': label_val
        })
        self._add_net_name(entry['name'])

    def _connection_exists(self, ref_a: str, pin_a: str, ref_b: str, pin_b: str) -> bool:
        """Check if a connection already exists between two pin endpoints."""
        for connection in self.connections:
            from_ref = connection['from']['ref']
            from_pin = connection['from']['pin']
            to_ref = connection['to']['ref']
            to_pin = connection['to']['pin']
            if (from_ref == ref_a and from_pin == pin_a and to_ref == ref_b and to_pin == pin_b):
                return True
            if (from_ref == ref_b and from_pin == pin_b and to_ref == ref_a and to_pin == pin_a):
                return True
        return False

    def _is_supply_net(self, net_name: str) -> bool:
        name = (net_name or '').strip().upper()
        if not name:
            return False
        supply_keywords = (
            'GND', 'AGND', 'DGND', 'IOGND', 'VCC', 'VDD', 'VSS', '3V3', '3V0', '5V', 'VIN', 'VREF'
        )
        if name in supply_keywords:
            return True
        if name.startswith(('VDD', 'VCC', '+')):
            return True
        return False

    def _validate_supply_net_isolation(self):
        """Warn if any pin appears on more than one supply net (e.g. J8.Pin1 on both VCC5 and VCC12)."""
        pin_to_nets: Dict[tuple, list] = {}  # (ref, pin) -> [net_name, ...]
        for net_key, entry in self.net_nodes.items():
            net_name = entry.get('name', net_key)
            if not self._is_supply_net(net_name):
                continue
            for node in entry.get('nodes') or []:
                key = (node['ref'], node['pin'])
                pin_to_nets.setdefault(key, []).append(net_name)
        for (ref, pin), nets in pin_to_nets.items():
            if len(nets) > 1:
                self.warnings.append(
                    f"Supply net conflict: {ref}.Pin{pin} appears on multiple supply nets: {', '.join(nets)}. "
                    "Fix the MD so each pin is on exactly one supply net."
                )

    def _should_ignore_net(self, net_name: Optional[str]) -> bool:
        if not net_name:
            return False
        return is_nc_identifier(net_name)

    def _build_connections_from_net_nodes(self):
        """Convert stored net memberships into explicit connections."""
        for net_key, entry in self.net_nodes.items():
            net_name = entry.get('name', net_key)
            if self._should_ignore_net(net_name):
                continue
            nodes = entry.get('nodes') or []
            if len(nodes) < 1:
                continue

            if self._is_supply_net(net_name):
                supply_ref = net_name.upper()
                for node in nodes:
                    ref = node['ref']
                    pin = node['pin']
                    label = node.get('label')
                    if self._connection_exists(ref, pin, supply_ref, '1'):
                        continue
                    self._add_connection(ref, pin, label, supply_ref, '1', None, net_name)
                continue

            anchor = nodes[0]
            for node in nodes[1:]:
                if self._connection_exists(anchor['ref'], anchor['pin'], node['ref'], node['pin']):
                    continue
                self._add_connection(
                    anchor['ref'], anchor['pin'], anchor.get('label'),
                    node['ref'], node['pin'], node.get('label'),
                    net_name, intermediate=True
                )

    def _parse_connections(self, content: str):
        """Parse inter-component connections."""
        pattern1 = r'Pin\s+(\d+)(?:\s*\(([^)]+)\))?:.*?(?:to|from)\s+([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?'
        pattern2 = r'([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?\s*[──]+\s*([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?'
        pattern3 = r'Pin\s+(\d+):\s+(\w+)\s*[──]+\s*([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?'
        pattern3b = r'Pin\s+(\d+)(?:\s*\(([^)]+)\))?:\s*([^─]+?)\s*[──]+\s*([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?'
        pattern4 = r'\[([A-Z]+\d+):.*?\]\s*[──]+\s*([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?'
        pattern5 = r'([A-Z]+\d+)\s*[──]+\s*([A-Z]+\d+)'
        pattern_supply = r'Pin\s+(\d+)(?:\s*\(([^)]+)\))?:.*?[──]+\s*(GND|VCC|VDD|VSS|AGND|DGND|IOGND|\+\d+\.?\d*V)(?:\s|$)'

        current_component = None
        known_refs = {c['reference'] for c in self.components}

        for raw_line in content.split('\n'):
            line = raw_line
            comp_header = re.match(r'###?\s+([A-Z]+\d+)\s*[-:]', line)
            if comp_header:
                current_component = comp_header.group(1)
                continue

            try:
                pin_identifier = None
                pin_label = None
                if current_component:
                    pad_match = re.match(r'\s*(PAD)\s*:', line, re.IGNORECASE)
                    if pad_match:
                        pin_identifier = 'PAD'
                    else:
                        pin_match = re.match(r'\s*Pin\s+(\d+)(?:\s*\(([^)]+)\))?', line, re.IGNORECASE)
                        if pin_match:
                            pin_identifier = pin_match.group(1)
                            pin_label = pin_match.group(2)
                    if pin_identifier and pin_label:
                        self._store_pin_label(current_component, pin_identifier, pin_label)

                line_net = None
                if current_component and pin_identifier:
                    net_refs = re.findall(r'(?:Net|Signal)\s+([A-Z_][A-Z0-9_]*)', line, re.IGNORECASE)
                    for net_name in net_refs:
                        self._register_net_node(net_name, current_component, pin_identifier, pin_label)
                    if net_refs:
                        line_net = net_refs[0]

                if current_component and 'Pin' in line and ('to' in line or 'from' in line):
                    matches = re.findall(pattern1, line, re.IGNORECASE)
                    for pin1, label1, comp2, pin2, label2 in matches:
                        self._add_connection(current_component, pin1, label1, comp2, pin2, label2, net_name=line_net)

                matches = re.findall(pattern2, line)
                for comp1, pin1, label1, comp2, pin2, label2 in matches:
                    self._add_connection(comp1, pin1, label1, comp2, pin2, label2,
                                         intermediate=True)

                if current_component:
                    if not line_net:
                        line_nets = re.findall(r'(?:Net|Signal)\s+([A-Z_][A-Z0-9_]*)', line, re.IGNORECASE)
                        line_net = line_nets[0] if line_nets else None
                    matches = re.findall(pattern3, line)
                    for pin1, label1, comp2, pin2, label2 in matches:
                        self._add_connection(current_component, pin1, label1, comp2, pin2, label2,
                                             net_name=line_net)
                    matches = re.findall(pattern3b, line)
                    for pin1, paren_label, text_label, comp2, pin2, label2 in matches:
                        from_label = paren_label or text_label.strip()
                        self._add_connection(current_component, pin1, from_label, comp2, pin2, label2,
                                             net_name=line_net)

                if current_component:
                    matches = re.findall(pattern_supply, line, re.IGNORECASE)
                    for pin1, label1, supply_net in matches:
                        self._add_connection(current_component, pin1, label1 or supply_net, supply_net, '1', None)

                matches = re.findall(pattern4, line)
                for comp1, comp2, pin2, label2 in matches:
                    from_match = re.search(rf'From\s+{comp1}', line, re.IGNORECASE)
                    if from_match:
                        self._add_connection(comp1, '1', None, comp2, pin2, label2)

                matches = re.findall(pattern5, line)
                for comp1, comp2 in matches:
                    # Skip if either "component" is actually a net name
                    # (e.g. "Net INV1 ─── U3" would wrongly treat INV1 as a part)
                    if comp1 in known_refs and comp2 in known_refs:
                        self._add_connection(comp1, '1', None, comp2, '1', None)

                # Catch-all: any CompRef.PinN on a line with current_component and pin_identifier
                # Handles direct references like "Pin 1 (LED1): J7.Pin2" with no separator
                if current_component and pin_identifier:
                    line_nets = re.findall(r'(?:Net|Signal)\s+([A-Z_][A-Z0-9_]*)', line, re.IGNORECASE)
                    catch_net = line_nets[0] if line_nets else None
                    all_refs = re.findall(r'([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?', line, re.IGNORECASE)
                    for comp2, pin2, label2 in all_refs:
                        if comp2.upper() != current_component.upper():
                            if not self._connection_exists(current_component, pin_identifier, comp2, pin2):
                                self._add_connection(current_component, pin_identifier, pin_label, comp2, pin2, label2 or None,
                                                     net_name=catch_net)

            except re.error:
                continue

        if len(self.connections) < 10:
            self._parse_connections_aggressive(content)

        self._build_connections_from_net_nodes()
        self._validate_supply_net_isolation()
        self._backfill_connection_labels()
        self._validate_parsed_connections(content)

    def _store_pin_label(self, ref: Optional[str], pin: Optional[str], label: Optional[str]):
        if not ref or not pin or not label:
            return
        bucket = self.pin_label_map.setdefault(ref.strip(), {})
        bucket.setdefault(pin.strip(), label.strip())

    def _lookup_pin_label(self, ref: Optional[str], pin: Optional[str]) -> Optional[str]:
        if not ref or not pin:
            return None
        return self.pin_label_map.get(ref.strip(), {}).get(pin.strip())

    def _backfill_connection_labels(self):
        if not self.connections or not self.pin_label_map:
            return
        for connection in self.connections:
            for side in ('from', 'to'):
                entry = connection.get(side)
                if not isinstance(entry, dict):
                    continue
                label = (entry.get('label') or '').strip()
                if label:
                    entry['label'] = label
                    continue
                ref = entry.get('ref')
                pin = entry.get('pin')
                fallback = self._lookup_pin_label(ref, pin)
                if fallback:
                    entry['label'] = fallback

    def _validate_parsed_connections(self, content: str):
        """Post-parse check: warn about component pin references that didn't become connections."""
        connected_pairs: Set[Tuple[str, str, str, str]] = set()
        for conn in self.connections:
            f = conn['from']
            t = conn['to']
            connected_pairs.add((f['ref'], f['pin'], t['ref'], t['pin']))
            connected_pairs.add((t['ref'], t['pin'], f['ref'], f['pin']))

        comp_refs = {c['reference'] for c in self.components}
        current_component = None
        ref_pattern = re.compile(r'([A-Z]+\d+)\.Pin\s*(\d+)', re.IGNORECASE)
        pin_line_pattern = re.compile(r'^\s*Pin\s+(\d+)(?:\s*\(([^)]+)\))?', re.IGNORECASE)
        missing_count = 0

        for raw_line in content.split('\n'):
            header = re.match(r'###?\s+([A-Z]+\d+)\s*[-:]', raw_line)
            if header:
                current_component = header.group(1)
                continue
            if not current_component:
                continue
            pin_match = pin_line_pattern.match(raw_line)
            if not pin_match:
                continue
            pin_id = pin_match.group(1)
            refs_on_line = ref_pattern.findall(raw_line)
            for target_ref, target_pin in refs_on_line:
                if target_ref.upper() == current_component.upper():
                    continue
                pair = (current_component, pin_id, target_ref, target_pin)
                if pair not in connected_pairs:
                    missing_count += 1
                    print(f"⚠ Parser gap: {current_component}.Pin{pin_id} → {target_ref}.Pin{target_pin} "
                          f"mentioned in MD but no connection created")
        if missing_count:
            print(f"⚠ Total parser gaps: {missing_count} connection(s) referenced in MD but not captured")

    def _add_connection(self, from_ref: str, from_pin: str, from_label: Optional[str],
                        to_ref: str, to_pin: str, to_label: Optional[str],
                        net_name: Optional[str] = None, intermediate: bool = False):
        """Store a parsed connection.  `intermediate=True` marks connections
        created from pattern2 CompRef─CompRef pairs within a net listing,
        as opposed to authoritative Pin N: declarations."""
        if not from_ref or not to_ref or not from_pin or not to_pin:
            return
        from_label_clean = (from_label or '').strip() or self._lookup_pin_label(from_ref, from_pin)
        to_label_clean = (to_label or '').strip() or self._lookup_pin_label(to_ref, to_pin)
        self.connections.append({
            'from': {
                'ref': from_ref,
                'pin': from_pin.strip(),
                'label': from_label_clean.strip() if from_label_clean else None
            },
            'to': {
                'ref': to_ref,
                'pin': to_pin.strip(),
                'label': to_label_clean.strip() if to_label_clean else None
            },
            'net': net_name.strip() if isinstance(net_name, str) and net_name.strip() else None,
            'intermediate': intermediate
        })

    def _parse_connections_aggressive(self, content: str):
        """Fallback connection parsing when patterns miss data."""
        for raw_line in content.split('\n'):
            if 'BILL OF MATERIALS' in raw_line or '```' in raw_line:
                continue

            comp_pin_refs = re.findall(r'([A-Z]+\d+)\.Pin\s*(\d+)(?:\s*\(([^)]+)\))?', raw_line)
            if len(comp_pin_refs) >= 2:
                for i in range(len(comp_pin_refs) - 1):
                    comp1, pin1, label1 = comp_pin_refs[i]
                    comp2, pin2, label2 = comp_pin_refs[i + 1]
                    exists = any(
                        c['from']['ref'] == comp1 and c['from']['pin'] == pin1 and
                        c['to']['ref'] == comp2 and c['to']['pin'] == pin2
                        for c in self.connections
                    )
                    if not exists:
                        self._add_connection(comp1, pin1, label1, comp2, pin2, label2)

    def _infer_package(self, ref: str, value: str) -> str:
        """Guess package when not explicitly provided."""
        value_upper = value.upper()

        if 'SMD' in value_upper or '0805' in value_upper:
            return '0805'
        if '0603' in value_upper:
            return '0603'
        if '1206' in value_upper:
            return '1206'
        if 'SOIC' in value_upper:
            return 'SOIC'
        if 'TSSOP' in value_upper:
            return 'TSSOP'
        if 'SOT' in value_upper:
            return 'SOT'
        if 'LFCSP' in value_upper or 'QFN' in value_upper:
            return 'LFCSP'
        if 'DIP' in value_upper or 'DIL' in value_upper:
            return 'DIP'
        if 'SMA' in value_upper or 'DO-' in value_upper:
            return 'SMA'
        if 'ELECTROLYTIC' in value_upper:
            return 'ELECTROLYTIC'
        return '0805'
    
    def _find_component_context(self, ref: str, content: str) -> str:
        """Find context/value for a component reference"""
        # Search for lines mentioning this component
        # Escape the ref in case it contains special regex characters
        safe_ref = re.escape(ref)
        
        for line in content.split('\n'):
            if ref in line:
                # Extract description after the component
                parts = line.split(ref)
                if len(parts) > 1:
                    after = parts[1].strip()
                    # Clean up
                    after = after.split('\n')[0]
                    after = after.split('(')[0]
                    after = after.strip(':- ')
                    if after:
                        return after[:50]  # Limit length
        
        return f"Component {ref}"
    
    # (second duplicate _parse_nets definition removed)


class EagleSCRGenerator:
    """Generate Eagle CAD .SCR script files"""
    
    def __init__(self, schematic_data: Dict, library_parser: EagleLibraryParser, pin_overrides: Optional[Dict[str, Dict[str, str]]] = None):
        self.data = schematic_data
        self.lib_parser = library_parser
        self.component_placements = {}
        self.net_segments = {}
        self.component_values: Dict[str, str] = {}
        self.pin_overrides = pin_overrides if pin_overrides is not None else {}
        self.pin_errors: List[Dict] = []
        self.override_usage: List[Dict] = []
        
    def generate(self, output_file: str) -> Tuple[bool, str]:
        """Generate the .SCR file"""
        scr_commands = []
        self.component_placements = {}
        self.pin_errors = []
        self.component_values = {}
        
        # Header
        scr_commands.append("# Eagle CAD Schematic Script")
        scr_commands.append("# Generated by Eagle Schematic Generator")
        scr_commands.append("")
        scr_commands.append("GRID INCH 0.1;")
        scr_commands.append("")
        
        # Collect all unique libraries needed
        libraries_used = set()
        matched_components = []
        unmatched_components = []
        
        # First pass: identify components and libraries
        component_data = []
        for idx, component in enumerate(self.data['components']):
            ref = component['reference']
            value = component['value']
            pkg = component.get('package', '')

            # Find matching library component
            lib_component = self.lib_parser.find_component(ref, value, pkg)

            if lib_component:
                library = lib_component['library']
                deviceset = lib_component['deviceset']
                device = lib_component['device']
                full_path = lib_component['full_path']
                
                libraries_used.add((library, full_path))
                
                component_data.append({
                    'ref': ref,
                    'value': value,
                    'library': library,
                    'deviceset': deviceset,
                    'device': device,
                    'pins': lib_component['pins'],
                    'idx': idx
                })
                
                matched_components.append(ref)
                self.component_values[ref] = value
            else:
                unmatched_components.append(f"{ref} ({value})")
        
        # Add USE commands for all libraries
        scr_commands.append("# Load required libraries")
        for library, lib_path in sorted(libraries_used):
            scr_commands.append(f"USE '{lib_path}';")
        scr_commands.append("")
        
        # Add components with correct syntax
        scr_commands.append("# Add components")
        grid_spacing = 2.0
        components_per_row = 5
        
        for comp in component_data:
            ref = comp['ref']
            value = comp['value']
            library = comp['library']
            deviceset = comp['deviceset']
            device = comp['device']
            idx = comp['idx']
            
            # Position calculation
            row = idx // components_per_row
            col = idx % components_per_row
            x_pos = col * grid_spacing
            y_pos = -row * grid_spacing
            
            self.component_placements[ref] = {
                'x': x_pos,
                'y': y_pos,
                'library': library,
                'deviceset': deviceset,
                'device': device,
                'pins': comp['pins']
            }
            
            # Correct ADD command syntax for Eagle
            # Format: ADD deviceset@library X Y [R angle]
            if device:  # Has a specific device/package variant
                scr_commands.append(f"ADD {deviceset}{device}@{library} '{ref}' R0 ({x_pos} {y_pos});")
            else:
                scr_commands.append(f"ADD {deviceset}@{library} '{ref}' R0 ({x_pos} {y_pos});")
            
            # Set value ONLY for passive components (R, C, L) - they support user-defined values
            # Do NOT set values for ICs, connectors, etc.
            if ref.startswith(('R', 'C', 'L')) and value and not value.startswith(('DAC', 'DC', 'op-amp', 'Filter', 'Protection', 'OR-ing')):
                scr_commands.append(f"VALUE '{ref}' '{value}';")
        
        scr_commands.append("")
        
        # Add nets with proper gate and pin syntax
        scr_commands.append("# Create net connections")
        
        # Build net map - group connections that should be on same net
        net_groups = {}  # Maps a set of connected pins to a net
        nets_by_name = {}
        net_counter = 1
        net_alias_map: Dict[str, str] = {}
        supply_net_elements: Dict[str, ET.Element] = {}
        supply_net_elements: Dict[str, ET.Element] = {}
        nets_created = 0
        nets_skipped = 0
        
        # First pass: validate all connections
        valid_connections = []
        
        for connection in self.data['connections']:
            from_ref = connection['from']['ref']
            from_pin = connection['from']['pin']
            from_label = connection['from'].get('label')
            to_ref = connection['to']['ref']
            to_pin = connection['to']['pin']
            to_label = connection['to'].get('label')
            
            # Check if both components exist in placements
            if from_ref not in self.component_placements:
                nets_skipped += 1
                scr_commands.append(f"# SKIPPED: {from_ref}.{from_pin} → {to_ref}.{to_pin}")
                scr_commands.append(f"#   Reason: Component '{from_ref}' not placed (not in BOM or failed to map)")
                continue
                
            if to_ref not in self.component_placements:
                nets_skipped += 1
                scr_commands.append(f"# SKIPPED: {from_ref}.{from_pin} → {to_ref}.{to_pin}")
                scr_commands.append(f"#   Reason: Component '{to_ref}' not placed (not in BOM or failed to map)")
                continue
            
            from_placement = self.component_placements[from_ref]
            to_placement = self.component_placements[to_ref]
            
            # Get available pins
            from_pins = from_placement.get('pins', {})
            to_pins = to_placement.get('pins', {})
            
            # Resolve requested pins against component pin maps
            from_pin_key, from_pin_info = self._resolve_pin_mapping(from_ref, from_pins, from_pin, from_label)
            if from_pin_key is None:
                nets_skipped += 1
                self._record_pin_error(from_ref, from_pin, from_label, from_pins)
                available = ', '.join(sorted(from_pins.keys())[:10]) if from_pins else 'Unknown'
                scr_commands.append(f"# SKIPPED: {from_ref}.{from_pin} → {to_ref}.{to_pin}")
                scr_commands.append(f"#   Reason: {from_ref} doesn't have pin {self._format_pin_display(from_pin, from_label)}")
                scr_commands.append(f"#   Available pins: {available}")
                continue
            
            to_pin_key, to_pin_info = self._resolve_pin_mapping(to_ref, to_pins, to_pin, to_label)
            if to_pin_key is None:
                nets_skipped += 1
                self._record_pin_error(to_ref, to_pin, to_label, to_pins)
                available = ', '.join(sorted(to_pins.keys())[:10]) if to_pins else 'Unknown'
                scr_commands.append(f"# SKIPPED: {from_ref}.{from_pin} → {to_ref}.{to_pin}")
                scr_commands.append(f"#   Reason: {to_ref} doesn't have pin {self._format_pin_display(to_pin, to_label)}")
                scr_commands.append(f"#   Available pins: {available}")
                continue
            
            connection_net = (connection.get('net') or '').strip() if isinstance(connection.get('net'), str) else ''
            net_name = ''
            if connection_net:
                net_key = connection_net.upper()
                net_name = net_alias_map.get(net_key)
                if not net_name:
                    formatted = self._format_net_name(connection_net)
                    if not formatted:
                        formatted = f"N${net_counter}"
                        net_counter += 1
                    net_alias_map[net_key] = formatted
                    net_name = formatted
            else:
                net_name = f"N${net_counter}"
                net_counter += 1
            
            # Get pin information from component
            # Build pin specification using resolved keys
            from_pin_spec = self._get_pin_spec(from_ref, from_pin_key, from_pin_info)
            to_pin_spec = self._get_pin_spec(to_ref, to_pin_key, to_pin_info)
            
            if from_pin_spec and to_pin_spec:
                # NET command: NET 'netname' part1 pin1 part2 pin2
                net_cmd = f"NET '{net_name}' {from_pin_spec} {to_pin_spec};"
                scr_commands.append(net_cmd)
                nets_created += 1
            else:
                nets_skipped += 1
                scr_commands.append(f"# SKIPPED: {from_ref}.{from_pin} → {to_ref}.{to_pin} (pin spec failed)")
        
        scr_commands.append(f"# NET Summary: {nets_created} created, {nets_skipped} skipped")
        
        # Add helpful note if J5 connections look wrong
        if nets_skipped > 0:
            scr_commands.append("#")
            scr_commands.append("# TROUBLESHOOTING TIPS:")
            scr_commands.append("# - Check component pin counts match your MD file connections")
            scr_commands.append("# - J5 (barrel jack) typically has 2-3 pins, not 4,5,6,12,13,14")
            scr_commands.append("# - Your MD file may have wrong component references (J5 vs U1/U2)")
            scr_commands.append("# - Fix MD file connections or component mappings")
        
        scr_commands.append("")
        scr_commands.append("GRID LAST;")
        scr_commands.append("WINDOW FIT;")
        
        # Write to file with UTF-8 encoding to handle special characters
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write('\n'.join(scr_commands))
            
            # Also write a JSON debug file with component matching details
            debug_file = output_file.replace('.scr', '_debug.json')
            debug_data = {
                'matched_components': [],
                'unmatched_components': unmatched_components,
                'component_placements': {}
            }
            
            for comp in component_data:
                debug_data['matched_components'].append({
                    'reference': comp['ref'],
                    'value': comp['value'],
                    'library': comp['library'],
                    'deviceset': comp['deviceset'],
                    'device': comp['device']
                })
            
            for ref, placement in self.component_placements.items():
                debug_data['component_placements'][ref] = {
                    'library': placement['library'],
                    'deviceset': placement['deviceset'],
                    'device': placement['device'],
                    'position': {'x': placement['x'], 'y': placement['y']},
                    'pins': list(placement['pins'].keys()) if placement['pins'] else []
                }
            
            with open(debug_file, 'w') as f:
                json.dump(debug_data, f, indent=2)
            
            status_msg = f"Successfully generated SCR file!\n\n"
            status_msg += f"Output: {output_file}\n"
            status_msg += f"Debug info: {debug_file}\n\n"
            status_msg += f"Matched components: {len(matched_components)}\n"
            if unmatched_components:
                status_msg += f"\nUnmatched components ({len(unmatched_components)}):\n"
                status_msg += '\n'.join(f"  - {c}" for c in unmatched_components)
            
            return True, status_msg
        
        except Exception as e:
            import traceback
            error_msg = f"Error writing SCR file: {str(e)}\n{traceback.format_exc()}"
            return False, error_msg
    
    def _get_pin_spec(self, ref: str, pin: str, pin_info: Optional[Dict]) -> str:
        """Get proper pin specification for NET command"""
        if not pin:
            pin = '1'
        # For simple components (R, C, L, D, LED) use direct pin reference
        if ref.startswith(('R', 'C', 'L', 'D', 'LED')):
            return f"'{ref}' '{pin}'"
        
        # Most connectors (J) can use the direct pin or pad name as well
        if ref.startswith('J'):
            return f"'{ref}' '{pin}'"
        
        # For components with explicit pin mapping, include gate/pad info when available
        if pin_info:
            gate = (pin_info.get('gate') or '').strip()
            pad = (pin_info.get('pad') or pin).strip()
            if gate and gate not in ('A', 'G$1'):
                return f"'{ref}' '{gate}' '{pad}'"
            return f"'{ref}' '{pad}'"
        
        # Fallback
        return f"'{ref}' '{pin}'"

    def _format_net_name(self, net_label: Optional[str]) -> str:
        """Create an Eagle-safe net name from arbitrary text."""
        if not net_label:
            return ''
        cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', net_label.strip())
        cleaned = re.sub(r'_+', '_', cleaned)
        cleaned = cleaned.strip('_')
        if not cleaned:
            return ''
        if cleaned[0].isdigit():
            cleaned = f"N_{cleaned}"
        return cleaned[:32]

    def _resolve_connector_pin_scr(self, ref: str, pin_map: Dict, requested_pin: str) -> Tuple[Optional[str], Optional[Dict]]:
        """Direct connector resolution: MD pin number → library pin by number/pad/position."""
        if not pin_map or not requested_pin or not requested_pin.isdigit():
            return None, None
        if requested_pin in pin_map:
            return requested_pin, pin_map[requested_pin]
        for key, value in pin_map.items():
            pad = str(value.get('pad') or '').strip()
            if not pad:
                continue
            for pt in pad.split():
                if pt == requested_pin:
                    return key, value
        for variant in (f"P${requested_pin}", f"-{requested_pin}"):
            if variant in pin_map:
                return variant, pin_map[variant]
        idx = int(requested_pin)
        ordered = self._order_connector_pins(pin_map)
        if 1 <= idx <= len(ordered):
            key = ordered[idx - 1]
            return key, pin_map[key]
        if len(pin_map) == 1:
            key = next(iter(pin_map))
            return key, pin_map[key]
        return None, None

    def _resolve_pin_mapping(self, ref: str, pin_map: Dict, requested_pin: str, pin_label: Optional[str]) -> Tuple[Optional[str], Optional[Dict]]:
        """Determine which pin identifier to use (numeric or named)"""
        requested_pin = (requested_pin or '').strip()
        if not pin_map:
            return requested_pin or None, None
        # Manual overrides first
        overrides = self.pin_overrides.get(ref, {})
        for cand in [c for c in [requested_pin, (pin_label or '').strip()] if c]:
            override_target = overrides.get(cand)
            if override_target:
                resolved = self._match_pin_name(pin_map, override_target)
                if resolved:
                    self.override_usage.append({
                        'ref': ref,
                        'requested': cand,
                        'mapped_to': resolved
                    })
                    return resolved, pin_map[resolved]
        # Connectors: direct number-to-number matching
        if ref.startswith('J'):
            result = self._resolve_connector_pin_scr(ref, pin_map, requested_pin)
            if result[0] is not None:
                return result
        # Passives (R, C, L): MD Pin 1/2 must map to distinct physical pins by position
        if ref.startswith(('R', 'C', 'L')) and requested_pin and requested_pin.isdigit():
            idx = int(requested_pin)
            ordered = self._order_connector_pins(pin_map)
            if 1 <= idx <= len(ordered):
                key = ordered[idx - 1]
                return key, pin_map[key]
        # Prioritize exact pin_label match (e.g. 1Y, 2Y, VCC)
        if pin_label:
            label_clean = pin_label.strip()
            if label_clean in pin_map:
                return label_clean, pin_map[label_clean]
            for k in pin_map:
                if k.upper() == label_clean.upper():
                    return k, pin_map[k]
            # Tilde-aware: Eagle uses ~ for active-low (2~A vs 2A)
            label_no_tilde = label_clean.replace('~', '').upper()
            for k in pin_map:
                if k.replace('~', '').upper() == label_no_tilde:
                    return k, pin_map[k]
        # ICs/transistors: match MD pin number to Eagle pad (physical pin) first
        if ref.startswith(('U', 'Q')) and requested_pin and requested_pin.isdigit():
            for key, value in pin_map.items():
                pad = str(value.get('pad') or '').strip()
                if not pad:
                    continue
                for pt in pad.split():
                    if pt == requested_pin or (pt.isdigit() and (pt.lstrip('0') or '0') == (requested_pin.lstrip('0') or '0')):
                        return key, value
        candidates = []
        if requested_pin:
            candidates.append(requested_pin)
        if pin_label:
            label_clean = pin_label.strip()
            if label_clean and label_clean not in candidates:
                candidates.append(label_clean)
            label_upper = label_clean.upper().replace(' ', '') if label_clean else ''
            if label_upper in ('+12V', '12V', 'VCC12', 'VCC5', '5V', '3V3'):
                candidates.extend(['VCC', 'VIN', 'V+', 'INPUT'])
            elif label_upper in ('GND', 'AGND', 'DGND', 'PGND'):
                candidates.extend(['GND', 'VSS', 'GROUND'])
        # Try exact matches first
        for cand in candidates:
            if cand in pin_map:
                return cand, pin_map[cand]
        # Case-insensitive comparison
        for cand in candidates:
            upper = cand.upper()
            for key, value in pin_map.items():
                if key.upper() == upper:
                    return key, value
        # Match by pad number (Eagle uses P$1, P$2 etc.; MD uses 1, 2)
        if requested_pin and requested_pin.isdigit():
            pin_num = requested_pin.lstrip('0') or '0'
            for key, value in pin_map.items():
                pad = str(value.get('pad') or '').strip()
                if not pad:
                    continue
                pad_tokens = pad.split()
                for pt in pad_tokens:
                    pt_norm = pt.lstrip('0') or pt
                    if pad == requested_pin or pt == requested_pin or pt_norm == pin_num:
                        return key, value
                    if pt == f"P${requested_pin}":
                        return key, value
            # P$N style: requested "1" -> "P$1"
            p_dollar = f"P${requested_pin}"
            if p_dollar in pin_map:
                return p_dollar, pin_map[p_dollar]
            # ICs with descriptive names (1CLR, 2D): match "1" to pins starting with "1"
            prefix_matches = [k for k in pin_map if k.startswith(requested_pin) and
                             (len(k) == len(requested_pin) or (len(k) > len(requested_pin) and not k[len(requested_pin)].isdigit()))]
            if len(prefix_matches) == 1:
                return prefix_matches[0], pin_map[prefix_matches[0]]
            if prefix_matches:
                # Prefer pin that matches pin_label (e.g. 3Y when label is "3Y")
                if pin_label:
                    label_clean = pin_label.strip()
                    for key in prefix_matches:
                        if key == label_clean or key.upper() == label_clean.upper():
                            return key, pin_map[key]
                for key in sorted(prefix_matches):
                    pad = str(pin_map.get(key, {}).get('pad') or '').strip()
                    if pad == requested_pin:
                        return key, pin_map[key]
                return prefix_matches[0], pin_map[prefix_matches[0]]
        # Connectors (J): MD Pin 1 = first physical pin, Pin 2 = second, etc.
        if ref.startswith('J') and requested_pin and requested_pin.isdigit():
            idx = int(requested_pin)
            ordered = self._order_connector_pins(pin_map)
            if 1 <= idx <= len(ordered):
                key = ordered[idx - 1]
                return key, pin_map[key]
        # Pin labels with / or - (e.g. 1REXT/CEXT): normalize and match
        if pin_label:
            norm_label = re.sub(r'[/\s\-]+', '', pin_label.upper())
            for key in pin_map:
                norm_key = re.sub(r'[/\s\-]+', '', key.upper())
                if norm_label == norm_key:
                    return key, pin_map[key]
                if len(norm_label) >= 3 and (norm_key.startswith(norm_label) or norm_label.startswith(norm_key)):
                    return key, pin_map[key]
        return None, None

    def _order_connector_pins(self, pin_map: Dict) -> List[str]:
        """Return pin keys sorted by physical position (1, 2, 3...). Used for connectors and passives."""
        def sort_key(item):
            k, v = item
            pad = str(v.get('pad') or '').strip().split()[0] if v.get('pad') else ''
            if pad and pad.isdigit():
                return (0, int(pad))
            if k.startswith('P$') and len(k) > 2 and k[2:].isdigit():
                return (0, int(k[2:]))
            if k.isdigit():
                return (0, int(k))
            if k.startswith('-') and len(k) > 1 and k[1:].isdigit():
                return (0, int(k[1:]))
            m = re.search(r'(\d+)\s*$', str(k))
            if m:
                return (0, int(m.group(1)))
            return (1, k)
        return [k for k, _ in sorted(pin_map.items(), key=sort_key)]

    def _format_pin_display(self, pin: str, pin_label: Optional[str]) -> str:
        if pin_label:
            return f"{pin} ({pin_label})"
        return pin

    def _match_pin_name(self, pin_map: Dict, target: str) -> Optional[str]:
        """Helper to match override target against available pin names"""
        if not target or not pin_map:
            return None
        if target in pin_map:
            return target
        upper = target.upper()
        for key in pin_map.keys():
            if key.upper() == upper:
                return key
        return None

    def _record_pin_error(self, ref: str, pin: str, pin_label: Optional[str], pin_map: Dict):
        available_pins = sorted(pin_map.keys()) if pin_map else []
        self.pin_errors.append({
            'ref': ref,
            'pin': (pin or '').strip(),
            'label': (pin_label or '').strip(),
            'available': available_pins,
            'value': self.component_values.get(ref, ''),
            'library': self.component_placements.get(ref, {}).get('library'),
            'deviceset': self.component_placements.get(ref, {}).get('deviceset'),
            'device': self.component_placements.get(ref, {}).get('device')
        })


class EagleSchematicWriter:
    """Generate Eagle CAD .sch XML files with embedded libraries and wire routing"""
    
    DEFAULT_GRID_SETTINGS = {
        'distance': '0.1',      # Primary grid spacing (in unitdist units)
        'unitdist': 'inch',
        'unit': 'inch',
        'style': 'lines',
        'multiple': '1',
        'display': 'yes',       # Show the grid when the schematic opens
        'altdistance': '0.05',  # Alternate grid for fine routing
        'altunitdist': 'inch',
        'altunit': 'inch'
    }

    POWER_PIN_KEYWORDS = (
        'VCC', 'VDD', 'DVDD', 'AVDD', 'PVDD', 'IOVDD', 'VEE', 'VNEG', 'VIN', 'VBUS',
        'VUSB', 'VBAT', 'VIO', 'VREF', 'CPVDD', 'VCORE', '3V3', '5V', '1V8', '2V5',
        'VOUT', 'LDO', 'CPVSS', 'CPVDD', 'CPVNEG'
    )
    GROUND_PIN_KEYWORDS = (
        'GND', 'PGND', 'AGND', 'DGND', 'BGND', 'VSS', 'VSSA', 'VSSD', 'VSSN',
        'VSSP', 'GATE_GND', 'EP(', 'PAD('
    )

    def __init__(self, schematic_data: Dict, library_parser: EagleLibraryParser, pin_overrides: Optional[Dict[str, Dict[str, str]]] = None, grid_settings: Optional[Dict[str, str]] = None):
        self.data = schematic_data
        self.lib_parser = library_parser
        self.pin_overrides = pin_overrides if pin_overrides is not None else {}
        self.component_value_map = self._build_component_value_map()
        self.component_placements = {}
        self.occupied_positions: set = set()
        self.occupied_regions: List[Dict[str, float]] = []
        self.route_tracks: List[Tuple[float, float, float]] = []
        self.vertical_tracks: List[Tuple[float, float, float]] = []
        self.pin_errors: List[Dict] = []
        self.override_usage: List[Dict] = []
        self.supply_symbols = {}  # Track supply symbols: {net_name: part_ref}
        self.supply_counter = 1  # Counter for unique supply symbol references
        self.supply_details: Dict[str, Dict] = {}
        self.supply_symbol_lookup: Dict[str, Dict[str, str]] = {}
        self.supply_bus_info: Dict[str, Dict] = {}
        self.expected_pin_links: Dict[Tuple[int, str], Dict] = {}
        self.connected_pin_links: Dict[Tuple[int, str], Dict] = {}
        self.missing_pin_links: List[Dict] = []
        effective_grid = grid_settings or self.DEFAULT_GRID_SETTINGS
        self.grid_settings = {key: str(value) for key, value in effective_grid.items()}
        self.grid_step = self._compute_grid_step()
        self.available_supply_devicesets = self._collect_available_supply_devicesets()
        # CRITICAL: Large clearance needed to prevent horizontal nets from routing through pin areas
        self.routing_clearance = max(self.grid_step * 20.0, 50.0)  # 50.8mm minimum
        self.bus_track_offset = max(self.grid_step * 6.0, 18.0)
        # Keep every coordinate inside Eagle's legal drawing area (~40 in / 1000 mm)
        self.coordinate_limit = max(self.grid_step * 400.0, 1000.0)
        self.supply_connected_nodes: Dict[Tuple[str, str], str] = {}
        self.pin_escape_usage: Dict[Tuple[float, int], int] = {}
        self.pin_horizontal_usage: Dict[Tuple[str, float, int], int] = {}
        
        # NEW: Unified wire segment registry for collision-free routing
        # Use LARGE clearance to prevent ERC "close but unconnected" warnings
        self.wire_registry = WireSegmentRegistry(
            grid_step=self.grid_step,
            min_clearance=max(self.grid_step * 8.0, 20.0)  # 8 grid steps or 20mm minimum between nets
        )
        self.supply_bus_positions: Dict[str, float] = {}  # Track supply bus X positions
        self.all_pin_y_coords: Set[float] = set()  # Global pin Y coordinates
        self.component_pin_columns: Dict[str, List[float]] = {}  # Track columns used per component
        self.per_pin_supply_instances: List[Dict] = []  # Per-pin supply symbol instances
        self.debug_messages: List[str] = []  # Diagnostic messages for the GUI

    def _build_component_value_map(self) -> Dict[str, str]:
        """Create quick lookup for component values by reference."""
        values: Dict[str, str] = {}
        components = []
        if isinstance(self.data, dict):
            components = self.data.get('components', []) or []
        for comp in components:
            if not isinstance(comp, dict):
                continue
            ref = (comp.get('reference') or '').strip()
            if not ref:
                continue
            values[ref] = comp.get('value', '') or ''
        return values

    def _get_component_value(self, ref: Optional[str]) -> str:
        if not ref:
            return ''
        return self.component_value_map.get(ref, '')
    
    def _is_supply_ref(self, ref: Optional[str]) -> bool:
        """Determine whether the provided reference represents a supply net label."""
        if not ref:
            return False
        ref = ref.strip()
        if not ref:
            return False
        if not ref[0].isalpha():
            return True
        upper = ref.upper()
        # Exact matches for common supplies
        if upper in ('GND', 'AGND', 'DGND', 'PGND', 'IOGND',
                      'VCC', 'VDD', 'VSS', 'VEE', 'VIN', 'VREF',
                      'VOUT', 'COM', '0V'):
            return True
        # Supply prefixes that may include voltage digits (VCC5, VCC12, VDD3V3)
        if upper.startswith(('VCC', 'VDD', 'VSS', 'VEE')):
            return True
        # Voltage rail patterns like 3V3, 5V, 12V
        if re.match(r'^\d+V\d*$', upper):
            return True
        # All-uppercase short names without digits (legacy path)
        if ref.isupper() and len(ref) < 10 and not any(c.isdigit() for c in ref):
            return True
        return False

    def _build_pin_error(self, ref: str, pin: str, label: Optional[str], error_msg: str,
                         pin_map: Optional[Dict] = None) -> Dict:
        """Build an enriched pin error dict with available pins and library info."""
        available_pins = sorted(pin_map.keys()) if pin_map else []
        placement = self.component_placements.get(ref, {})
        return {
            'ref': ref,
            'pin': (pin or '').strip(),
            'label': (label or '').strip(),
            'value': self._get_component_value(ref),
            'available': available_pins,
            'library': placement.get('library'),
            'deviceset': placement.get('deviceset'),
            'device': placement.get('device'),
            'error': error_msg
        }

    def _identify_supply_nets(self):
        """Scan connections and identify which supply nets must be instantiated."""
        supply_nets: Dict[str, str] = {}
        connections = self.data.get('connections') if isinstance(self.data, dict) else None
        if not connections:
            return

        for connection in connections:
            if not isinstance(connection, dict):
                continue
            for endpoint in ('from', 'to'):
                entry = connection.get(endpoint) or {}
                ref = entry.get('ref')
                if not self._is_supply_ref(ref):
                    continue
                clean = (ref or '').strip()
                if not clean:
                    continue
                normalized = re.sub(r'\s+', '', clean.upper())
                if normalized not in supply_nets:
                    supply_nets[normalized] = clean

        for net_upper, net_original in supply_nets.items():
            if net_upper in self.supply_symbol_lookup:
                continue
            part_ref = f"SUPPLY{self.supply_counter}"
            self.supply_counter += 1
            self.supply_symbols[net_original] = part_ref
            self.supply_symbol_lookup[net_upper] = {
                'name': net_original,
                'part': part_ref
            }

    def _collect_available_supply_devicesets(self) -> Set[str]:
        supply_lib = self.lib_parser.library_xml_cache.get('supply1') if self.lib_parser else None
        devicesets: Set[str] = set()
        if supply_lib is None:
            return devicesets
        for deviceset_elem in supply_lib.findall(".//deviceset"):
            name = deviceset_elem.get('name')
            if name:
                devicesets.add(name.upper())
        return devicesets

    def _supply_deviceset_exists(self, deviceset: Optional[str]) -> bool:
        if not deviceset:
            return False
        if not self.available_supply_devicesets:
            self.available_supply_devicesets = self._collect_available_supply_devicesets()
        return deviceset.upper() in self.available_supply_devicesets

    def _fallback_supply_deviceset(self, normalized: str) -> str:
        normalized = normalized or ''
        fallback_candidates: List[str] = []
        if normalized.startswith('-'):
            fallback_candidates.extend(['-5V', '-12V'])
        elif normalized.startswith('+') or normalized.startswith('V') or (normalized[:1].isdigit()):
            fallback_candidates.extend(['+3V3', '+05V', 'VCC', 'VDD'])
        else:
            fallback_candidates.extend(['VCC', 'VDD', '+3V3'])
        fallback_candidates.append('GND')
        for candidate in fallback_candidates:
            if self._supply_deviceset_exists(candidate):
                return candidate
        if self.available_supply_devicesets:
            return next(iter(self.available_supply_devicesets))
        return 'GND'

    def _map_supply_deviceset(self, net_name: str) -> str:
        """Map a net label to the appropriate supply symbol deviceset."""
        normalized = (net_name or '').strip().upper()
        normalized = normalized.replace(' ', '')

        mapping = {
            'GND': 'GND',
            'AGND': 'AGND',
            'DGND': 'DGND',
            'PGND': 'PGND',
            'IOGND': 'GND',
            'COM': 'GND',
            '0V': 'GND',
            '+3V3': '+3V3',
            '3V3': '+3V3',
            '+3.3V': '+3V3',
            '3.3V': '+3V3',
            '+5V': '+05V',
            '5V': '+05V',
            '5.0V': '+05V',
            '+12V': '+12V',
            '12V': '+12V',
            '-5V': '-5V',
            '-5.0V': '-5V',
            'NEG5V': '-5V',
            '-12V': '-12V',
            '+1V8': '+1V8',
            '1V8': '+1V8',
            '+1.8V': '+1V8',
            '1.8V': '+1V8',
            '+1V2': '+1V2',
            '+1.2V': '+1V2',
            '1V2': '+1V2',
            'VCC5': '+05V',
            'VCC5V': '+05V',
            'VCC12': '+12V',
            'VCC12V': '+12V',
            'VCC3V3': '+3V3',
            'VDD5': '+05V',
            'VDD3V3': '+3V3',
            'VDD12': '+12V',
            'VIN': 'VCC',
            'VBUS': '+05V',
            'VUSB': '+05V',
            'VBAT': '+05V',
            'BAT': '+05V',
            'VREG': 'VCC'
        }

        candidate = mapping.get(normalized, normalized or 'GND')
        if not self._supply_deviceset_exists(candidate):
            candidate = self._fallback_supply_deviceset(normalized)
        return candidate

    def _resolve_supply_symbol(self, net_label: Optional[str]) -> Optional[Dict[str, str]]:
        if not net_label:
            return None
        normalized = re.sub(r'\s+', '', net_label.strip().upper())
        if not normalized:
            return None
        return self.supply_symbol_lookup.get(normalized)
        
    def _lookup_supply_symbol_info(self, deviceset: str) -> Tuple[str, str, float, float]:
        """Fetch gate, pin, and relative pin coordinates for the given supply deviceset."""
        default_gate = 'G$1'
        default_pin = deviceset
        supply_lib = self.lib_parser.library_xml_cache.get('supply1')
        if supply_lib is None:
            return default_gate, default_pin, 0.0, 0.0

        deviceset_elem = supply_lib.find(f".//deviceset[@name='{deviceset}']")
        if deviceset_elem is None:
            return default_gate, default_pin, 0.0, 0.0

        gate_elem = deviceset_elem.find('.//gate')
        if gate_elem is None:
            return default_gate, default_pin, 0.0, 0.0

        gate_name = gate_elem.get('name', default_gate)
        symbol_name = gate_elem.get('symbol')

        pin_name = default_pin
        pin_x = 0.0
        pin_y = 0.0
        if symbol_name:
            symbol_elem = supply_lib.find(f".//symbol[@name='{symbol_name}']")
            if symbol_elem is not None:
                pin_elem = symbol_elem.find('.//pin')
                if pin_elem is not None:
                    pin_name = pin_elem.get('name', default_pin)
                    try:
                        pin_x = float(pin_elem.get('x', 0.0))
                    except (TypeError, ValueError):
                        pin_x = 0.0
                    try:
                        pin_y = float(pin_elem.get('y', 0.0))
                    except (TypeError, ValueError):
                        pin_y = 0.0

        return gate_name or default_gate, pin_name or default_pin, pin_x, pin_y

    def _resolve_part_technology(self, lib_component: Dict, component: Dict) -> Optional[str]:
        """Determine the Eagle technology name that should be referenced for this part."""
        if not isinstance(lib_component, dict):
            return None

        raw_names = lib_component.get('technologies') or []
        tech_names = [(name or '').strip() for name in raw_names]
        if not tech_names:
            return None

        # If the library defines a blank technology, Eagle can use it implicitly
        if any(not name for name in tech_names):
            return None

        component_value = ''
        if isinstance(component, dict):
            component_value = (component.get('value') or '').strip()
        value_upper = component_value.upper()

        if value_upper:
            details = lib_component.get('technology_details') or []
            match = self._match_technology_value(value_upper, details)
            if match:
                return match
            for name in tech_names:
                if name and name.upper() in value_upper:
                    return name

        return tech_names[0] if tech_names else None

    def _match_technology_value(self, value_upper: str, technology_details: List[Dict]) -> Optional[str]:
        """Attempt to match a component value against technology metadata."""
        for detail in technology_details:
            tech_name = (detail.get('name') or '').strip()
            attr_list = detail.get('attributes') or []
            for attr in attr_list:
                attr_value = (attr.get('value') or '').strip()
                if not attr_value:
                    continue
                attr_upper = attr_value.upper()
                if attr_upper in value_upper or value_upper in attr_upper:
                    return tech_name or None
            if tech_name:
                tech_upper = tech_name.upper()
                if tech_upper in value_upper or value_upper in tech_upper:
                    return tech_name
        return None

    def _compute_grid_step(self) -> float:
        """Compute primary grid spacing in millimeters for coordinate snapping."""
        distance = self.grid_settings.get('distance', '0.1') or '0.1'
        unit = (self.grid_settings.get('unitdist') or self.grid_settings.get('unit') or 'inch').lower()
        try:
            distance_value = float(distance)
        except ValueError:
            distance_value = 0.1

        unit_to_mm = {
            'inch': 25.4,
            'mm': 1.0,
            'mil': 0.0254
        }
        multiplier = unit_to_mm.get(unit, 25.4)
        step = distance_value * multiplier
        if step <= 0:
            step = 2.54  # default to 0.1 inch
        return step

    def _prepare_expected_connections(self):
        """Cache every MD connection that should place a pinref in the schematic."""
        self.expected_pin_links = {}
        self.connected_pin_links = {}
        self.missing_pin_links = []

        connections = self.data.get('connections') or []
        for idx, connection in enumerate(connections):
            if not isinstance(connection, dict):
                continue
            for side in ('from', 'to'):
                entry = connection.get(side) or {}
                ref = (entry.get('ref') or '').strip()
                if not ref or self._is_supply_ref(ref):
                    continue
                other_side_name = 'to' if side == 'from' else 'from'
                other_entry = connection.get(other_side_name) or {}
                other_ref = (other_entry.get('ref') or '').strip()
                if self._is_supply_ref(other_ref):
                    net_hint = other_ref or 'supply net'
                elif other_ref:
                    other_pin_desc = (other_entry.get('pin') or other_entry.get('label') or '?').strip()
                    net_hint = f"{other_ref}.{other_pin_desc}"
                else:
                    net_hint = 'unknown net'
                self.expected_pin_links[(idx, side)] = {
                    'ref': ref,
                    'requested_pin': (entry.get('pin') or '').strip(),
                    'label': (entry.get('label') or '').strip(),
                    'other_side': other_entry,
                    'net_hint': net_hint
                }

    def _record_connected_pin(self, conn_index: int, side: str, ref: str,
                              requested_pin: Optional[str], resolved_pin: Optional[str],
                              label: Optional[str], net_name: Optional[str]):
        """Mark that a specific MD connection endpoint produced a pinref."""
        key = (conn_index, side)
        info = self.expected_pin_links.get(key)
        if not info:
            return
        if resolved_pin:
            info['resolved_pin'] = resolved_pin
        if requested_pin:
            info['requested_pin'] = requested_pin.strip()
        if label:
            info['label'] = label.strip()
        if net_name:
            info['net_name'] = net_name
        self.connected_pin_links[key] = {
            'ref': ref,
            'requested_pin': requested_pin,
            'resolved_pin': resolved_pin,
            'label': label,
            'net_name': net_name
        }

    def _mark_supply_connected_pin(self, ref: Optional[str], pin_name: Optional[str], net_name: Optional[str]):
        """Remember that a component pin is already tied to a named supply net."""
        if not ref or not pin_name or not net_name:
            return
        key = (ref.strip(), pin_name.strip().upper())
        if not key[0] or not key[1]:
            return
        self.supply_connected_nodes[key] = net_name.strip()

    def _group_nodes_share_supply(self, node_keys: List[Tuple[str, str]]) -> Tuple[bool, Optional[str]]:
        """Return True if every node in the group already connects to the same supply net."""
        if not node_keys:
            return False, None
        supply_name: Optional[str] = None
        for ref, pin in node_keys:
            key = (ref, pin.upper())
            net = self.supply_connected_nodes.get(key)
            if not net:
                return False, None
            if supply_name is None:
                supply_name = net
            elif supply_name != net:
                return False, None
        return True, supply_name

    def _write_sch_connection_debug(self, output_file: str) -> Optional[str]:
        """Write a debug file listing all SCH connections and verify against MD.
        Returns the debug file path if written, else None."""
        try:
            base = str(output_file).rsplit('.', 1)[0] if '.' in str(output_file) else str(output_file)
            debug_path = f"{base}_sch_connections_debug.txt"
            lines: List[str] = []
            lines.append("=== SCH Connection Debug ===")
            lines.append("")

            # 1. Build net → [(ref, pin), ...] from connected_pin_links
            net_to_pins: Dict[str, List[Tuple[str, str]]] = {}
            for key, info in self.connected_pin_links.items():
                net = (info.get('net_name') or '').strip()
                ref = (info.get('ref') or '').strip()
                pin = (info.get('requested_pin') or info.get('resolved_pin') or '').strip()
                if not net:
                    net = "(no net)"
                if ref and pin:
                    net_to_pins.setdefault(net, []).append((ref, pin))

            # Deduplicate per net (same ref.pin can appear from multiple connections)
            for net in net_to_pins:
                net_to_pins[net] = list(dict.fromkeys(net_to_pins[net]))

            lines.append("--- Nets in SCH (net name → pins) ---")
            for net in sorted(net_to_pins.keys(), key=lambda n: (0 if n.startswith('N$') else 1, n)):
                pins = sorted(net_to_pins[net], key=lambda p: (p[0], p[1]))
                pin_str = ', '.join(f"{r}.Pin{p}" for r, p in pins)
                lines.append(f"  {net}: {pin_str}")
            lines.append("")

            # 2. MD connections for reference
            connections = self.data.get('connections') or []
            lines.append("--- MD File Connections (expected) ---")
            for idx, conn in enumerate(connections):
                if not isinstance(conn, dict):
                    continue
                fr = conn.get('from') or {}
                to = conn.get('to') or {}
                f_ref = (fr.get('ref') or '').strip()
                f_pin = (fr.get('pin') or '').strip()
                t_ref = (to.get('ref') or '').strip()
                t_pin = (to.get('pin') or '').strip()
                if self._is_supply_ref(f_ref) or self._is_supply_ref(t_ref):
                    continue
                if f_ref and f_pin and t_ref and t_pin:
                    lines.append(f"  [{idx}] {f_ref}.Pin{f_pin} → {t_ref}.Pin{t_pin}")
            lines.append("")

            # 3. Verify: for each MD connection, from and to should be on same net
            lines.append("--- Verification (MD vs SCH) ---")
            mismatches: List[str] = []
            for idx, conn in enumerate(connections):
                if not isinstance(conn, dict):
                    continue
                fr = conn.get('from') or {}
                to = conn.get('to') or {}
                f_ref = (fr.get('ref') or '').strip()
                f_pin = (fr.get('pin') or '').strip()
                t_ref = (to.get('ref') or '').strip()
                t_pin = (to.get('pin') or '').strip()
                if self._is_supply_ref(f_ref) or self._is_supply_ref(t_ref):
                    continue
                if not (f_ref and f_pin and t_ref and t_pin):
                    continue

                from_key = (idx, 'from')
                to_key = (idx, 'to')
                from_info = self.connected_pin_links.get(from_key, {})
                to_info = self.connected_pin_links.get(to_key, {})

                from_net = (from_info.get('net_name') or '').strip()
                to_net = (to_info.get('net_name') or '').strip()

                if not from_net and not to_net:
                    continue  # both missing - already in missing report
                if from_net != to_net:
                    mismatches.append(
                        f"  MISMATCH: {f_ref}.Pin{f_pin} → {t_ref}.Pin{t_pin}  "
                        f"(from on net '{from_net or 'MISSING'}', to on net '{to_net or 'MISSING'}')"
                    )

            if mismatches:
                lines.append("  ⚠ ERRORS (from and to on different nets):")
                lines.extend(mismatches)
                for m in mismatches:
                    self.debug_messages.append(m.strip())
            else:
                lines.append("  ✓ All verified connections have from and to on same net.")

            with open(debug_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines))
            return debug_path
        except Exception as e:
            self.debug_messages.append(f"Could not write connection debug: {e}")
            return None

    def _validate_connection_coverage(self):
        """Add pin errors for any MD connections that never produced pinrefs."""
        if not self.expected_pin_links:
            return

        existing_keys = set()
        for err in self.pin_errors:
            if not isinstance(err, dict):
                continue
            key_ref = err.get('ref')
            key_pin = (err.get('pin') or '').upper()
            key_label = (err.get('label') or '').upper()
            existing_keys.add((key_ref, key_pin, key_label))

        for key, expected in self.expected_pin_links.items():
            if key in self.connected_pin_links:
                continue
            ref = expected.get('ref')
            if not ref:
                continue
            requested_pin = expected.get('requested_pin') or ''
            label = expected.get('label') or ''
            net_hint = expected.get('net_name') or expected.get('net_hint') or 'unknown net'
            pin_desc = expected.get('resolved_pin') or requested_pin or label or '?'
            key_tuple = (ref, requested_pin.upper(), label.upper())
            if key_tuple in existing_keys:
                continue
            other_entry = expected.get('other_side') or {}
            placement = self.component_placements.get(ref, {})
            pin_map = placement.get('pins', {})
            missing_entry = {
                'ref': ref,
                'pin': requested_pin,
                'label': label,
                'value': self._get_component_value(ref),
                'available': sorted(pin_map.keys()) if pin_map else [],
                'library': placement.get('library'),
                'deviceset': placement.get('deviceset'),
                'device': placement.get('device'),
                'error': f"Connection missing for {ref} pin {pin_desc} -> {net_hint}"
            }
            if other_entry:
                missing_entry['target_ref'] = other_entry.get('ref')
                missing_entry['target_pin'] = other_entry.get('pin')
                missing_entry['target_label'] = other_entry.get('label')
            self.pin_errors.append(missing_entry)
            self.missing_pin_links.append(missing_entry)
            existing_keys.add(key_tuple)

    def _detect_unreferenced_power_pins(self):
        if not self.component_placements:
            return
        usage_map = self._build_pin_usage_map()
        for ref, placement in self.component_placements.items():
            pin_map = placement.get('pins') or {}
            if not pin_map:
                continue
            used_tokens = usage_map.get(ref, set())
            for pin_name, pin_info in pin_map.items():
                if not self._is_power_critical_pin(pin_name, pin_info):
                    continue
                if self._pin_tokens(pin_name, pin_info) & used_tokens:
                    continue
                if self._has_valid_override(ref, {'pin': pin_name}):
                    continue
                already_reported = any(
                    err.get('ref') == ref and (err.get('pin') or '').upper() == pin_name.upper()
                    for err in self.pin_errors
                )
                if already_reported:
                    continue
                self.pin_errors.append({
                    'ref': ref,
                    'pin': pin_name,
                    'label': pin_name,
                    'value': self._get_component_value(ref),
                    'library': placement.get('library'),
                    'deviceset': placement.get('deviceset'),
                    'device': placement.get('device'),
                    'available': sorted(pin_map.keys()),
                    'error': ("Power pin {0} has no connection in the Markdown file. "
                              "Add an explicit net or document it as NC.").format(pin_name)
                })

    def _build_pin_usage_map(self) -> Dict[str, Set[str]]:
        usage: Dict[str, Set[str]] = {}
        for info in self.connected_pin_links.values():
            ref = info.get('ref')
            if not ref:
                continue
            bucket = usage.setdefault(ref, set())
            for key in ('resolved_pin', 'requested_pin', 'label'):
                bucket.update(self._tokenize_pin_identifier(info.get(key)))
        return usage

    def _tokenize_pin_identifier(self, identifier: Optional[str]) -> Set[str]:
        tokens: Set[str] = set()
        if not identifier:
            return tokens
        trimmed = identifier.strip()
        if not trimmed:
            return tokens
        upper = trimmed.upper()
        tokens.add(upper)
        compact = re.sub(r'[^A-Z0-9]+', '', upper)
        if compact:
            tokens.add(compact)
        for part in re.split(r'[^A-Z0-9]+', upper):
            if part:
                tokens.add(part)
        return tokens

    def _pin_tokens(self, pin_name: str, pin_info: Optional[Dict]) -> Set[str]:
        tokens = self._tokenize_pin_identifier(pin_name)
        if isinstance(pin_info, dict):
            pad = pin_info.get('pad')
            tokens.update(self._tokenize_pin_identifier(str(pad) if pad else None))
        return tokens

    def _is_power_critical_pin(self, pin_name: str, pin_info: Optional[Dict]) -> bool:
        if not pin_name:
            return False
        upper = pin_name.upper()
        if any(keyword in upper for keyword in self.GROUND_PIN_KEYWORDS):
            return False
        if any(keyword in upper for keyword in self.POWER_PIN_KEYWORDS):
            return True
        if isinstance(pin_info, dict):
            direction = (pin_info.get('direction') or '').lower()
            if direction in ('pwr', 'sup'):
                if any(keyword in upper for keyword in self.GROUND_PIN_KEYWORDS):
                    return False
                return True
        return False

    def _has_valid_override(self, ref: str, issue: Dict) -> bool:
        overrides = self.pin_overrides.get(ref, {})
        for key in (issue.get('pin'), issue.get('label')):
            if key and key in overrides:
                return True
        return False

    def _snap_to_grid(self, value: float) -> float:
        """Snap a coordinate to the configured grid spacing."""
        step = self.grid_step or 2.54
        if step <= 0:
            step = 2.54
        return round(value / step) * step

    def _snap_to_routing_grid(self, value: float) -> float:
        """Snap a coordinate to a fine routing grid for wire placement."""
        # Use 1/10th of the main grid for routing to allow many more channels
        routing_step = (self.grid_step or 2.54) / 10.0
        if routing_step <= 0:
            routing_step = 0.254
        return round(value / routing_step) * routing_step

    def _clamp_routing_coordinate(self, value: float) -> float:
        """Keep routing coordinates inside Eagle's drawable window."""
        limited = max(-self.coordinate_limit, min(self.coordinate_limit, value))
        return self._snap_to_routing_grid(limited)

    def generate(self, output_file: str) -> Tuple[bool, str]:
        """Generate the .sch file with full Eagle XML structure"""
        try:
            # Create root structure
            root = ET.Element('eagle', version="7.4.0")
            drawing = ET.SubElement(root, 'drawing')
            
            # Add settings
            settings = ET.SubElement(drawing, 'settings')
            ET.SubElement(settings, 'setting', alwaysvectorfont="no")
            ET.SubElement(settings, 'setting', verticaltext="up")
            
            # Add grid
            ET.SubElement(drawing, 'grid', **self.grid_settings)
            
            # Add layers (standard Eagle layers)
            self._add_layers(drawing)
            
            # Create schematic
            schematic = ET.SubElement(drawing, 'schematic', xreflabel="%F%N/%S.%C%R", xrefpart="/%S.%C%R")
            
            # Reset supply bookkeeping for this generation
            self.supply_symbols = {}
            self.supply_counter = 1
            self.supply_details = {}
            self.supply_symbol_lookup = {}
            self.supply_bus_info = {}
            
            # Reset routing state for this generation
            self.route_tracks = []
            self.vertical_tracks = []
            self.occupied_regions = []
            self.component_placements = {}
            self.occupied_positions = set()
            self.supply_bus_positions = {}
            self.all_pin_y_coords = set()
            self.component_pin_columns = {}
            self.pin_escape_usage = {}
            self.pin_horizontal_usage = {}
            
            # Reset wire registry for clean routing with LARGE clearances
            self.wire_registry = WireSegmentRegistry(
                grid_step=self.grid_step,
                min_clearance=max(self.grid_step * 8.0, 20.0)  # 8 grid steps or 20mm minimum
            )
            self.per_pin_supply_instances = []  # Per-pin supply symbols

            # Identify supply nets before building libraries/parts
            self._identify_supply_nets()

            # Add libraries with embedded definitions
            libraries_elem, used_libraries = self._add_libraries(schematic)
            
            # Add attributes (optional)
            attributes = ET.SubElement(schematic, 'attributes')
            
            # Add variantdefs (optional)
            variantdefs = ET.SubElement(schematic, 'variantdefs')
            
            # Add classes (optional)
            classes = ET.SubElement(schematic, 'classes')
            ET.SubElement(classes, 'class', number="0", name="default", width="0", drill="0")
            
            # Add parts
            parts_elem = ET.SubElement(schematic, 'parts')
            matched_components = []
            unmatched_components = []
            self.occupied_positions = set()
            self.occupied_regions = []
            self.route_tracks = []
            self.vertical_tracks = []
            self.supply_connected_nodes = {}
            self.pin_escape_usage = {}
            self.pin_horizontal_usage = {}
            self.all_pin_y_coords = set()  # Track all pin Y-coordinates globally to avoid horizontal lanes at pin rows
            self.component_pin_columns = {}  # Track which X columns are used by each component's pins
            self.supply_bus_positions = {}  # Remember bus X positions per supply net to prevent overlaps
            self.category_lane_settings = {}
            self.global_lane_registry = {}
            
            for component in self.data['components']:
                ref = component['reference']
                value = component['value']
                pkg = component.get('package', '')
                lib_component = self.lib_parser.find_component(ref, value, pkg)
                
                if lib_component:
                    part_attrs = {
                        'name': ref,
                        'library': lib_component['library'],
                        'deviceset': lib_component['deviceset'],
                        'device': lib_component['device'],
                        'value': value
                    }
                    technology_name = self._resolve_part_technology(lib_component, component)
                    if technology_name:
                        part_attrs['technology'] = technology_name
                    ET.SubElement(parts_elem, 'part', **part_attrs)
                    matched_components.append(ref)
                else:
                    unmatched_components.append(f"{ref} ({value})")
            
            if unmatched_components:
                self.debug_messages.append(f"\n⚠ UNMATCHED COMPONENTS ({len(unmatched_components)}):")
                for uc in unmatched_components:
                    self.debug_messages.append(f"  ✗ {uc} — ALL connections to this component will be DROPPED")
                self.debug_messages.append("")
            
            # Add supply symbol parts
            for supply_net, part_ref in self.supply_symbols.items():
                # Map supply net names to Eagle supply symbols. Only use a symbol when the net name matches an available device.
                deviceset_map = {
                    'GND': 'GND',
                    'AGND': 'AGND',
                    'DGND': 'DGND',
                    'IOGND': 'GND',  # Use GND for IOGND alias
                    'VCC': 'VCC',
                    'VDD': 'VDD',
                    'VSS': 'GND',  # VSS is typically ground in CMOS
                    '+3.3V': '+3V3',
                    '+5V': '+05V',
                }
                deviceset = deviceset_map.get(supply_net)
                has_symbol = bool(deviceset and self._supply_deviceset_exists(deviceset))
                supply_detail = {
                    'deviceset': deviceset if has_symbol else None,
                    'gate': None,
                    'pin': None,
                    'offset_x': 0.0,
                    'offset_y': 0.0,
                    'net': supply_net,
                    'has_symbol': has_symbol
                }
                if has_symbol:
                    gate_name, pin_name, pin_x, pin_y = self._lookup_supply_symbol_info(deviceset)
                    supply_detail.update({
                        'gate': gate_name,
                        'pin': pin_name,
                        'offset_x': pin_x,
                        'offset_y': pin_y
                    })
                    part = ET.SubElement(parts_elem, 'part',
                                         name=part_ref,
                                         library='supply1',
                                         deviceset=deviceset,
                                         device='',
                                         value=supply_net)
                self.supply_details[part_ref] = supply_detail
            
            # Add sheets with instances and nets
            sheets = ET.SubElement(schematic, 'sheets')
            sheet = ET.SubElement(sheets, 'sheet')
            
            # Add plain (text/graphics)
            plain = ET.SubElement(sheet, 'plain')
            
            # Add instances (component placements)
            instances = ET.SubElement(sheet, 'instances')
            self._add_instances(instances, matched_components)
            
            # Add supply symbol instances (one per supply net)
            self._add_supply_instances(instances)

            # Prepare separated lane bands for routing
            self._initialize_lane_categories()
            
            # Add busses (optional)
            busses = ET.SubElement(sheet, 'busses')
            
            # Add nets with wires
            nets = ET.SubElement(sheet, 'nets')
            self._prepare_expected_connections()
            self._add_nets(nets, busses)
            
            # Write to file with proper XML declaration
            tree = ET.ElementTree(root)
            ET.indent(tree, space="  ")
            
            with open(output_file, 'wb') as f:
                f.write(b'<?xml version="1.0" encoding="utf-8"?>\n')
                f.write(b'<!DOCTYPE eagle SYSTEM "eagle.dtd">\n')
                tree.write(f, encoding='utf-8', xml_declaration=False)
            
            # Post-generation audit: compare expected vs actual connections
            total_conns = len(self.data.get('connections', []))
            connected_count = len(self.connected_pin_links)
            expected_count = len(self.expected_pin_links)
            missing = expected_count - connected_count
            self.debug_messages.append(f"\n=== Post-Generation Audit ===")
            self.debug_messages.append(f"MD connections: {total_conns}")
            self.debug_messages.append(f"Expected endpoints: {expected_count}")
            self.debug_messages.append(f"Connected endpoints: {connected_count}")
            self.debug_messages.append(f"Missing endpoints: {missing}")
            if missing > 0:
                self.debug_messages.append(f"⚠ {missing} connection endpoint(s) from MD were NOT wired into the schematic!")
                for key, exp in self.expected_pin_links.items():
                    if key not in self.connected_pin_links:
                        ref = exp.get('ref', '?')
                        pin = exp.get('requested_pin', '?')
                        label = exp.get('label', '')
                        hint = exp.get('net_hint', '?')
                        self.debug_messages.append(f"  MISSING: {ref}.Pin{pin} ({label}) → {hint}")
            self.debug_messages.append(f"=============================\n")

            # Write SCH connection debug file and verify against MD
            debug_path = self._write_sch_connection_debug(output_file)
            if debug_path:
                self.debug_messages.append(f"\nConnection debug written: {debug_path}")

            status_msg = f"Successfully generated SCH file!\n\n"
            status_msg += f"Output: {output_file}\n\n"
            if debug_path:
                status_msg += f"Connection debug: {debug_path}\n\n"
            status_msg += f"Matched components: {len(matched_components)}\n"
            if unmatched_components:
                status_msg += f"\nUnmatched components ({len(unmatched_components)}):\n"
                status_msg += '\n'.join(f"  - {c}" for c in unmatched_components)
            if missing > 0:
                status_msg += f"\n\n⚠ WARNING: {missing} connection endpoint(s) not wired!"
            
            return True, status_msg
            
        except Exception as e:
            import traceback
            error_msg = f"Error writing SCH file: {str(e)}\n{traceback.format_exc()}"
            return False, error_msg
    
    def _add_layers(self, drawing):
        """Add standard Eagle layer definitions"""
        layers = ET.SubElement(drawing, 'layers')
        # Essential schematic layers
        layer_defs = [
            (91, "Nets", "2", "1", "yes", "yes"),
            (92, "Busses", "1", "1", "yes", "yes"),
            (93, "Pins", "2", "1", "yes", "yes"),
            (94, "Symbols", "4", "1", "yes", "yes"),
            (95, "Names", "7", "1", "yes", "yes"),
            (96, "Values", "7", "1", "yes", "yes"),
            (97, "Info", "7", "1", "yes", "yes"),
            (98, "Guide", "6", "1", "yes", "yes"),
        ]
        for num, name, color, fill, visible, active in layer_defs:
            ET.SubElement(layers, 'layer', number=str(num), name=name, 
                         color=color, fill=fill, visible=visible, active=active)
    
    def _add_libraries(self, schematic) -> Tuple[ET.Element, set]:
        """Add library definitions with full embedded XML"""
        import copy
        libraries = ET.SubElement(schematic, 'libraries')
        used_libraries = set()
        
        for component in self.data['components']:
            ref = component['reference']
            value = component['value']
            pkg = component.get('package', '')
            lib_component = self.lib_parser.find_component(ref, value, pkg)
            
            if lib_component:
                lib_name = lib_component['library']
                if lib_name not in used_libraries:
                    used_libraries.add(lib_name)
                    
                    # Get cached library XML and deep copy it
                    if lib_name in self.lib_parser.library_xml_cache:
                        lib_elem_original = self.lib_parser.library_xml_cache[lib_name]
                        lib_elem = copy.deepcopy(lib_elem_original)
                        
                        # Ensure library has name attribute
                        if lib_elem.get('name') is None:
                            lib_elem.set('name', lib_name)
                        
                        libraries.append(lib_elem)

        # Ensure supply library is embedded when supply symbols are used
        if getattr(self, 'supply_symbols', None):
            supply_lib = 'supply1'
            if supply_lib not in used_libraries and supply_lib in self.lib_parser.library_xml_cache:
                lib_elem_original = self.lib_parser.library_xml_cache[supply_lib]
                lib_elem = copy.deepcopy(lib_elem_original)
                if lib_elem.get('name') is None:
                    lib_elem.set('name', supply_lib)
                libraries.append(lib_elem)
                used_libraries.add(supply_lib)
        
        return libraries, used_libraries

    def _ensure_pin_spacing_annotations(self, lib_component: Dict):
        """Annotate each pin with the nearest vertical neighbor spacing for keep-out tuning."""
        if not isinstance(lib_component, dict):
            return
        if lib_component.get('_pin_spacing_ready'):
            return
        pin_map = lib_component.get('pins')
        if not isinstance(pin_map, dict) or not pin_map:
            lib_component['_pin_spacing_ready'] = True
            return

        columns: Dict[float, List[Tuple[float, str]]] = {}
        for pin_name, info in pin_map.items():
            if not isinstance(info, dict):
                continue
            try:
                x_val = float(info.get('x', 0.0))
                y_val = float(info.get('y', 0.0))
            except (TypeError, ValueError):
                continue
            key = round(x_val, 3)
            columns.setdefault(key, []).append((y_val, pin_name))

        for entries in columns.values():
            if not entries:
                continue
            entries.sort(key=lambda item: item[0])
            count = len(entries)
            for idx, (y_val, name) in enumerate(entries):
                prev_diff = float('inf')
                next_diff = float('inf')
                if idx > 0:
                    prev_diff = abs(y_val - entries[idx - 1][0])
                if idx < count - 1:
                    next_diff = abs(entries[idx + 1][0] - y_val)
                pitch = min(prev_diff, next_diff)
                if pitch != float('inf') and name in pin_map and isinstance(pin_map[name], dict):
                    pin_map[name]['vertical_pitch'] = pitch

        lib_component['_pin_spacing_ready'] = True
    
    def _add_instances(self, instances_elem, matched_refs):
        """Add component instances with separate grids for large ICs and small passives."""
        
        # Separate components into large (ICs, connectors) and passive (R, C, L, D)
        large_refs = []
        passive_refs = []
        
        for ref in matched_refs:
            prefix = re.match(r'^([A-Z]+)', ref)
            prefix = prefix.group(1) if prefix else ''
            if prefix in ('R', 'C', 'L', 'D', 'LED'):
                passive_refs.append(ref)
            else:
                large_refs.append(ref)
        
        # Grid settings for large components (ICs, connectors, etc.)
        large_spacing_x = self.grid_step * 50  # Wide spacing for ICs to prevent overlap
        large_spacing_y = self.grid_step * 35
        large_per_row = 3
        
        # Grid settings for passive components (much tighter)
        passive_spacing_x = self.grid_step * 12  # Compact horizontal
        passive_spacing_y = self.grid_step * 8   # Compact vertical
        passive_per_row = 6
        
        # Place large components first (top area)
        for idx, ref in enumerate(large_refs):
            col = idx % large_per_row
            row = idx // large_per_row
            
            base_x = self._snap_to_grid(col * large_spacing_x)
            base_y = self._snap_to_grid(-row * large_spacing_y)
            
            self._place_component_instance(instances_elem, ref, base_x, base_y)
        
        # Calculate where passives should start (below the large components)
        large_rows = (len(large_refs) + large_per_row - 1) // large_per_row if large_refs else 0
        passive_start_y = -large_rows * large_spacing_y - self.grid_step * 15  # Gap below ICs
        
        # Place passive components (below large components, tighter grid)
        for idx, ref in enumerate(passive_refs):
            col = idx % passive_per_row
            row = idx // passive_per_row
            
            base_x = self._snap_to_grid(col * passive_spacing_x)
            base_y = self._snap_to_grid(passive_start_y - row * passive_spacing_y)
            
            self._place_component_instance(instances_elem, ref, base_x, base_y)
    
    def _place_component_instance(self, instances_elem, ref, base_x, base_y):
        """Place a single component instance at the given base position."""
        component = next((c for c in self.data['components'] if c['reference'] == ref), None)
        if not component:
            return
        lib_component = self.lib_parser.find_component(ref, component['value'], component.get('package', ''))
        if not lib_component:
            return

        self._ensure_pin_spacing_annotations(lib_component)

        span_w, span_h = self._estimate_component_span(lib_component)
        x_pos, y_pos = self._find_free_position(base_x, base_y, span_w, span_h)

        self.occupied_positions.add((round(x_pos, 2), round(y_pos, 2)))
        
        # Store placement for wire routing
        pins = lib_component['pins']
        self.component_placements[ref] = {
            'x': x_pos,
            'y': y_pos,
            'library': lib_component['library'],
            'deviceset': lib_component['deviceset'],
            'device': lib_component['device'],
            'pins': pins
        }

        # Log connector pin details
        if ref.startswith('J') or ref.startswith('U'):
            pins_with_coords = sum(1 for p in pins.values() if isinstance(p, dict) and 'x' in p and 'y' in p)
            self.debug_messages.append(
                f"  {ref}: {len(pins)} pins ({pins_with_coords} with coords), names={sorted(pins.keys())}"
            )
        
        # Collect all pin coordinates and register with wire registry
        pins = lib_component.get('pins', {})
        for pin_name, pin_info in pins.items():
            if isinstance(pin_info, dict):
                gx = float(pin_info.get('gate_x', 0))
                gy = float(pin_info.get('gate_y', 0))
                if 'y' in pin_info:
                    absolute_pin_y = y_pos + gy + float(pin_info['y'])
                    self.all_pin_y_coords.add(absolute_pin_y)
                if 'x' in pin_info and 'y' in pin_info:
                    absolute_pin_x = x_pos + gx + float(pin_info['x'])
                    absolute_pin_y = y_pos + gy + float(pin_info['y'])
                    self.wire_registry.add_pin_location(absolute_pin_x, absolute_pin_y)
        
        # Add instance for each gate, applying gate offsets for multi-gate parts
        deviceset_elem = lib_component.get('deviceset_elem')
        gate_names = lib_component.get('gate_names', [])

        if deviceset_elem is not None:
            gates = deviceset_elem.findall('.//gate')
            if not gates:
                gates = deviceset_elem.findall('gates/gate')
        else:
            gates = []

        if gates:
            for gate in gates:
                gate_name = gate.get('name', 'G$1')
                gate_offset_x = float(gate.get('x', 0))
                gate_offset_y = float(gate.get('y', 0))
                inst_x = self._snap_to_grid(x_pos + gate_offset_x)
                inst_y = self._snap_to_grid(y_pos + gate_offset_y)
                ET.SubElement(instances_elem, 'instance',
                            part=ref,
                            gate=gate_name,
                            x=str(inst_x),
                            y=str(inst_y))
            self._register_region(x_pos, y_pos, span_w, span_h)
            self.debug_messages.append(f"  {ref}: {len(gates)} gate(s) instantiated: {[g.get('name') for g in gates]}")
        elif gate_names:
            for gn in gate_names:
                ET.SubElement(instances_elem, 'instance',
                            part=ref,
                            gate=gn,
                            x=str(x_pos),
                            y=str(y_pos))
            self._register_region(x_pos, y_pos, span_w, span_h)
            self.debug_messages.append(f"  {ref}: {len(gate_names)} gate(s) from gate_names: {gate_names}")
        else:
            ET.SubElement(instances_elem, 'instance',
                        part=ref,
                        gate='G$1',
                        x=str(x_pos),
                        y=str(y_pos))
            self._register_region(x_pos, y_pos, span_w, span_h)
            self.debug_messages.append(f"  {ref}: fallback to single G$1 gate")
    
    def _add_supply_instances(self, instances_elem):
        """Add supply symbol instances, including per-pin supply symbols."""
        grid = self.grid_step or 2.54
        x_start = self._snap_to_grid(grid * 80)
        y_start = self._snap_to_grid(grid * 60)
        y_spacing = grid * 3

        # Place shared supply symbols (non per-pin) in a tidy column
        for idx, (supply_net, part_ref) in enumerate(self.supply_symbols.items()):
            y_pos = self._snap_to_grid(y_start - (idx * y_spacing))

            supply_info = self.supply_details.get(part_ref, {})
            if not supply_info.get('has_symbol', True):
                continue  # No visual symbol available for this supply net
            gate_name = supply_info.get('gate', 'G$1')
            pin_name = supply_info.get('pin', supply_net)
            deviceset = supply_info.get('deviceset', supply_net)

            pin_offset_x = supply_info.get('offset_x', 0.0)
            pin_offset_y = supply_info.get('offset_y', 0.0)

            contact_x = x_start
            contact_y = y_pos
            origin_x = self._snap_to_grid(contact_x - pin_offset_x)
            origin_y = self._snap_to_grid(contact_y - pin_offset_y)

            span_w, span_h = self._estimate_supply_span()
            origin_x, origin_y = self._find_free_position(origin_x, origin_y, span_w, span_h)
            contact_x = self._snap_to_grid(origin_x + pin_offset_x)
            contact_y = self._snap_to_grid(origin_y + pin_offset_y)

            self.component_placements[part_ref] = {
                'x': origin_x,
                'y': origin_y,
                'library': 'supply1',
                'deviceset': deviceset,
                'device': '',
                'pins': {pin_name: {'gate': gate_name, 'pad': '1', 'x': pin_offset_x, 'y': pin_offset_y}}
            }

            ET.SubElement(instances_elem, 'instance', part=part_ref, gate=gate_name, x=str(origin_x), y=str(origin_y))
            self.occupied_positions.add((round(origin_x, 2), round(origin_y, 2)))
            self._register_region(origin_x, origin_y, span_w, span_h)

        return
    
    def _compute_supply_offset(self, pin_info: Dict) -> Tuple[str, int, float, float]:
        """Heuristically determine where to place a supply symbol relative to a pin."""
        grid = self.grid_step or 2.54
        base_distance = max(grid * 4, grid)

        x_rel = pin_info.get('x', 0.0)
        y_rel = pin_info.get('y', 0.0)

        if abs(x_rel) > abs(y_rel):
            axis = 'x'
            direction = 1 if x_rel >= 0 else -1
            dx = base_distance * direction
            dy = 0.0
        else:
            axis = 'y'
            direction = 1 if y_rel >= 0 else -1
            dx = 0.0
            dy = base_distance * direction

        if dx == 0.0 and dy == 0.0:
            axis = 'y'
            direction = -1
            dy = -base_distance

        return axis, direction, dx, dy

    def _find_connected_ics(self, ref: str) -> List[str]:
        """Find which ICs this component connects to"""
        connected = []
        for connection in self.data.get('connections', []):
            from_ref = connection['from']['ref']
            to_ref = connection['to']['ref']
            
            if from_ref == ref and to_ref.startswith('U'):
                connected.append(to_ref)
            elif to_ref == ref and from_ref.startswith('U'):
                connected.append(from_ref)
        
        return list(set(connected))  # Remove duplicates
    
    def _estimate_component_span(self, lib_component: Dict) -> Tuple[float, float]:
        """Estimate the footprint size of a component for placement spacing."""
        pins = lib_component.get('pins', {}) if isinstance(lib_component, dict) else {}
        xs = []
        ys = []
        for info in pins.values():
            if not isinstance(info, dict):
                continue
            if 'x' in info:
                xs.append(float(info['x']))
            if 'y' in info:
                ys.append(float(info['y']))

        span_x = (max(xs) - min(xs)) if xs else 0.0
        span_y = (max(ys) - min(ys)) if ys else 0.0

        margin_x = self.grid_step * 8
        margin_y = self.grid_step * 6
        min_x = self.grid_step * 12
        min_y = self.grid_step * 10

        width = max(span_x + margin_x, min_x)
        height = max(span_y + margin_y, min_y)
        return width, height

    def _estimate_supply_span(self) -> Tuple[float, float]:
        """Return a conservative size for supply symbols."""
        side = max(self.grid_step * 8, 10.0)
        return side, side

    def _register_region(self, center_x: float, center_y: float, width: float, height: float):
        """Track occupied rectangular region for collision avoidance."""
        half_w = width / 2.0
        half_h = height / 2.0
        self.occupied_regions.append({
            'x': center_x,
            'y': center_y,
            'half_w': half_w,
            'half_h': half_h
        })
        # Also register with wire registry for routing collision detection
        self.wire_registry.add_keepout_zone(center_x, center_y, half_w, half_h)

    def _overlaps_region(self, center_x: float, center_y: float, half_w: float, half_h: float) -> bool:
        """Check if proposed placement overlaps any occupied region."""
        for region in self.occupied_regions:
            if (abs(center_x - region['x']) < (half_w + region['half_w']) and
                    abs(center_y - region['y']) < (half_h + region['half_h'])):
                return True
        return False

    def _horizontal_crosses_component(self, y_coord: float, x_start: float, x_end: float, clearance: float) -> bool:
        """Return True if a horizontal run would cross a component keep-out region."""
        if not self.occupied_regions:
            return False
        x1 = min(x_start, x_end)
        x2 = max(x_start, x_end)
        for region in self.occupied_regions:
            rx1 = region['x'] - region['half_w'] - clearance
            rx2 = region['x'] + region['half_w'] + clearance
            ry1 = region['y'] - region['half_h'] - clearance
            ry2 = region['y'] + region['half_h'] + clearance
            if y_coord >= ry1 and y_coord <= ry2 and x2 >= rx1 and x1 <= rx2:
                return True
        return False

    def _vertical_crosses_component(self, x_coord: float, y_start: float, y_end: float, clearance: float) -> bool:
        """Return True if a vertical run would cross a component keep-out region."""
        if not self.occupied_regions:
            return False
        y1 = min(y_start, y_end)
        y2 = max(y_start, y_end)
        for region in self.occupied_regions:
            rx1 = region['x'] - region['half_w'] - clearance
            rx2 = region['x'] + region['half_w'] + clearance
            ry1 = region['y'] - region['half_h'] - clearance
            ry2 = region['y'] + region['half_h'] + clearance
            if x_coord >= rx1 and x_coord <= rx2 and y2 >= ry1 and y1 <= ry2:
                return True
        return False

    def _pick_bus_offset_y(self, lane_y: float, x_start: float, x_end: float) -> float:
        """Choose a bus drawing Y coordinate that stays clear of components."""
        base_offsets = [self.bus_track_offset, -self.bus_track_offset]
        for multiplier in range(1, 6):
            for base in base_offsets:
                candidate = self._snap_to_routing_grid(lane_y + base * multiplier)
                if not self._horizontal_crosses_component(candidate, x_start, x_end, self.routing_clearance):
                    return self._clamp_routing_coordinate(candidate)
        fallback = self._snap_to_routing_grid(lane_y + self.bus_track_offset)
        return self._clamp_routing_coordinate(fallback)

    def _global_top_reference(self) -> float:
        if not self.occupied_regions:
            return self._snap_to_grid(-(self.grid_step or 2.54) * 80.0)
        top_edges = [region['y'] - region['half_h'] for region in self.occupied_regions]
        return self._snap_to_grid(min(top_edges) - max(self.grid_step * 12.0, 20.0))

    def _initialize_lane_categories(self):
        """Prepare separated horizontal bands for each net category."""
        top_reference = self._global_top_reference()
        band_spacing = max(self.grid_step * 160.0, 400.0)
        lane_step = max(self.grid_step * 40.0, 100.0)
        categories = ['supply', 'analog', 'digital', 'control', 'default']
        self.category_lane_settings = {}
        for idx, category in enumerate(categories):
            origin = self._snap_to_routing_grid(top_reference - idx * band_spacing)
            self.category_lane_settings[category] = {
                'origin': origin,
                'step': lane_step,
                'next_index': 0
            }
        self.global_lane_registry = {}

    def _categorize_net(self, net_name: Optional[str]) -> str:
        if not net_name:
            return 'default'
        upper = net_name.upper()
        if any(keyword in upper for keyword in ('GND', 'VSS', 'VDD', 'VIN', 'VOUT', 'BIAS', 'VBAT', 'VCC')):
            return 'supply'
        if any(keyword in upper for keyword in ('AUDIO', 'AN', 'LDO', 'DAC', 'ADC', 'REF', 'PLL')):
            return 'analog'
        if any(keyword in upper for keyword in ('NS', 'SCL', 'SDA', 'CLK', 'IO', 'BUS', 'I2C', 'SPI', 'DATA', 'ADR')):
            return 'digital'
        if any(keyword in upper for keyword in ('IRQ', 'RESET', 'CTRL', 'EN', 'PWREN', 'IRQ/IO', 'IRQ/IO1')):
            return 'control'
        return 'default'

    def _assign_global_lane(self, net_key: str, category: str) -> float:
        if not self.category_lane_settings:
            self._initialize_lane_categories()
        if category not in self.category_lane_settings:
            category = 'default'
        registry = self.global_lane_registry.setdefault(category, {})
        if net_key in registry:
            return registry[net_key]
        settings = self.category_lane_settings[category]
        lane_index = settings['next_index']
        settings['next_index'] += 1
        lane_y = self._snap_to_routing_grid(settings['origin'] - lane_index * settings['step'])
        registry[net_key] = lane_y
        return lane_y

    def _find_free_position(self, base_x: float, base_y: float, width: float, height: float) -> Tuple[float, float]:
        """Find a nearby grid position that avoids overlapping other components."""
        base_x = self._snap_to_grid(base_x)
        base_y = self._snap_to_grid(base_y)
        half_w = width / 2.0
        half_h = height / 2.0

        if not self._overlaps_region(base_x, base_y, half_w, half_h):
            return base_x, base_y

        step = max(self.grid_step * 10, width, height)

        for distance in range(1, 40):
            for dx in range(-distance, distance + 1):
                for dy in range(-distance, distance + 1):
                    if abs(dx) != distance and abs(dy) != distance:
                        continue

                    candidate_x = self._snap_to_grid(base_x + dx * step)
                    candidate_y = self._snap_to_grid(base_y + dy * step)
                    if not self._overlaps_region(candidate_x, candidate_y, half_w, half_h):
                        return candidate_x, candidate_y

        # As a fallback, push the component far enough along X to avoid crowding
        fallback_x = self._snap_to_grid(base_x + step * (len(self.occupied_regions) + 1))
        fallback_y = base_y
        return fallback_x, fallback_y

    def _select_lane_y(
        self,
        base_y: float,
        x_start: float,
        x_end: float,
        avoid_y: Optional[List[float]] = None,
        min_lane_offset: Optional[float] = None,
        net_id: Optional[str] = None
    ) -> float:
        """Choose a lane Y coordinate using the unified wire registry for collision-free routing."""
        base_y = self._clamp_routing_coordinate(base_y)
        x1 = min(x_start, x_end)
        x2 = max(x_start, x_end)
        
        # Use wire registry for collision-free lane selection
        effective_net_id = net_id or f"net_{len(self.route_tracks)}"
        
        # The wire registry handles all collision detection internally
        lane_y = self.wire_registry.find_free_horizontal_lane(
            effective_net_id, base_y, x1, x2,
            search_range=max(self.coordinate_limit * 0.5, 500.0)
        )
        
        # Clamp to valid coordinate range
        return self._clamp_routing_coordinate(lane_y)

    def _reserve_lane(self, lane_y: float, start_x: float, end_x: float, net_id: Optional[str] = None):
        """Record the horizontal extent of a routed lane."""
        x1 = min(start_x, end_x)
        x2 = max(start_x, end_x)
        self.route_tracks.append((lane_y, x1, x2))
        
        # Also register with wire registry for comprehensive collision tracking
        effective_net_id = net_id or f"h_lane_{len(self.route_tracks)}"
        self.wire_registry.register_segment(effective_net_id, x1, lane_y, x2, lane_y)

    def _reserve_vertical_path(self, x: float, y_start: float, y_end: float, target_lane_y: Optional[float] = None, 
                               component_ref: Optional[str] = None, net_id: Optional[str] = None) -> float:
        """Find a column for a vertical segment using the unified wire registry."""
        if abs(y_end - y_start) < 0.01:
            return self._snap_to_routing_grid(x)

        base_x = self._snap_to_routing_grid(x)
        y1 = min(y_start, y_end)
        y2 = max(y_start, y_end)
        
        # Use wire registry for collision-free vertical column selection
        effective_net_id = net_id or f"v_col_{len(self.vertical_tracks)}"
        
        candidate = self.wire_registry.find_free_vertical_column(
            effective_net_id, base_x, y1, y2,
            search_range=max(self.coordinate_limit * 0.3, 200.0)
        )
        
        # Clamp to coordinate limits
        candidate = max(-self.coordinate_limit, min(self.coordinate_limit, candidate))
        candidate = self._snap_to_routing_grid(candidate)
        
        # Register for backward compatibility with old tracking system
        self.vertical_tracks.append((candidate, y1, y2))
        
        # Register this column as used by this component
        if component_ref:
            if component_ref not in self.component_pin_columns:
                self.component_pin_columns[component_ref] = []
            self.component_pin_columns[component_ref].append(candidate)
        
        # Register with wire registry
        self.wire_registry.register_segment(effective_net_id, candidate, y1, candidate, y2)
        
        return candidate

    def _allocate_escape_slot(self, pin_y: float, direction: float) -> int:
        """Assign a unique escape index for pins leaving the same row/direction."""
        key = (round(pin_y, 2), 1 if direction >= 0 else -1)
        slot = self.pin_escape_usage.get(key, 0)
        self.pin_escape_usage[key] = slot + 1
        return slot

    def _allocate_horizontal_slot(self, component_ref: Optional[str], pin_info: Optional[Dict], direction: float) -> int:
        """Assign horizontal fan-out slots per component row to stagger exits."""
        if not component_ref or not isinstance(pin_info, dict):
            return 0
        try:
            pin_y_rel = float(pin_info.get('y', 0.0))
        except (TypeError, ValueError):
            pin_y_rel = 0.0
        key = (component_ref.strip(), round(pin_y_rel, 2), 1 if direction >= 0 else -1)
        slot = self.pin_horizontal_usage.get(key, 0)
        self.pin_horizontal_usage[key] = slot + 1
        return slot

    def _apply_horizontal_fanout(self, segment: ET.Element, component_ref: Optional[str], pin_info: Optional[Dict],
                                 current_x: float, current_y: float, axis_direction: Optional[Tuple[str, int]],
                                 net_id: Optional[str] = None) -> float:
        """Shift wires sideways before heading vertical to avoid stacked columns."""
        if not axis_direction or axis_direction[0] != 'x':
            return current_x
        direction = axis_direction[1]
        slot = self._allocate_horizontal_slot(component_ref, pin_info, direction)
        if slot <= 0:
            return current_x
        spacing = max(self.grid_step * 8.0, 20.0)  # Much larger horizontal spacing
        offset = direction * spacing * slot
        candidate = self._snap_to_routing_grid(current_x + offset)
        if abs(candidate - current_x) < 0.01:
            return current_x
        if self._horizontal_crosses_component(current_y, current_x, candidate, self.routing_clearance * 0.4):
            return current_x
        if abs(candidate) > self.coordinate_limit:
            candidate = max(-self.coordinate_limit, min(self.coordinate_limit, candidate))
            candidate = self._snap_to_routing_grid(candidate)
        ET.SubElement(segment, 'wire',
                      x1=str(current_x), y1=str(current_y),
                      x2=str(candidate), y2=str(current_y),
                      width="0.1524", layer="91")
        # Register segment with wire registry
        if net_id:
            self.wire_registry.register_segment(net_id, current_x, current_y, candidate, current_y)
        return candidate

    def _compute_departure_y(self, pin_y: float, lane_y: float, direction: float) -> float:
        """Pick an intermediate Y level to jog away from the pin row before heading horizontal."""
        distance = abs(lane_y - pin_y)
        min_escape = max(self.grid_step * 2.5, 6.0)
        slot = self._allocate_escape_slot(pin_y, direction)
        slot_spacing = max(self.grid_step * 1.25, 2.5)
        desired = min_escape + slot * slot_spacing
        # Do not overshoot the lane; leave a small buffer so vertical leg has room
        max_allowed = max(distance - self.grid_step * 0.75, self.grid_step * 0.5)
        offset = min(desired, max_allowed)
        if offset <= self.grid_step * 0.5:
            offset = min(distance * 0.45, max_allowed)
        if offset <= 0:
            return self._snap_to_grid(pin_y)
        stage = pin_y + direction * offset
        buffer = max(self.grid_step * 0.5, 1.0)
        if direction > 0:
            stage = min(stage, lane_y - buffer)
        else:
            stage = max(stage, lane_y + buffer)
        return self._snap_to_grid(stage)

    def _pin_keepout_distance(self) -> float:
        """Distance to pull wires away from pin centers to avoid ERC overlaps."""
        base = self.grid_step or 2.54
        distance = max(base * 3.0, base)
        return self._snap_to_grid(distance) or (base * 3.0)

    def _lane_pin_clearance(self) -> float:
        """Minimum vertical offset between a routing lane and the pins it serves."""
        keepout = self._pin_keepout_distance()
        # Keep horizontal rails well away from the pin rows to avoid obscuring circles
        return max(keepout * 1.5, self.grid_step * 8.0)

    def _ensure_lane_pin_gap(self, lane_y: float, pin_y_values: Optional[List[float]]) -> float:
        """Push a horizontal lane far enough away from its pin rows for visibility."""
        if not pin_y_values:
            return self._snap_to_routing_grid(lane_y)

        sanitized: List[float] = []
        seen: Set[float] = set()
        for value in pin_y_values:
            if value is None:
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            key = round(numeric, 4)
            if key in seen:
                continue
            seen.add(key)
            sanitized.append(numeric)

        if not sanitized:
            return self._snap_to_routing_grid(lane_y)

        clearance = self._lane_pin_clearance()
        lane = self._snap_to_routing_grid(lane_y)

        def too_close(candidate: float) -> bool:
            for pin_y in sanitized:
                if abs(candidate - pin_y) < clearance:
                    return True
            return False

        if not too_close(lane):
            return lane

        avg_pin = sum(sanitized) / len(sanitized)
        direction = 1.0 if lane >= avg_pin else -1.0
        shift = max(self.grid_step * 6.0, clearance)
        attempts = 0
        while attempts < 120 and too_close(lane):
            lane = self._snap_to_routing_grid(lane + direction * shift)
            lane = max(-self.coordinate_limit, min(self.coordinate_limit, lane))
            attempts += 1

        if too_close(lane):
            direction *= -1.0
            attempts = 0
            while attempts < 120 and too_close(lane):
                lane = self._snap_to_routing_grid(lane + direction * shift)
                lane = max(-self.coordinate_limit, min(self.coordinate_limit, lane))
                attempts += 1

        return lane

    def _format_net_name(self, net_label: Optional[str]) -> str:
        """Generate an Eagle-safe net identifier."""
        if not net_label:
            return ''
        cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', net_label.strip())
        cleaned = re.sub(r'_+', '_', cleaned).strip('_')
        if not cleaned:
            return ''
        if cleaned[0].isdigit():
            cleaned = f"N_{cleaned}"
        return cleaned[:32]

    def _derive_pin_orientation(self, pin_info: Optional[Dict]) -> Optional[Tuple[str, int]]:
        """Return (axis, direction) for a pin based on its rotation metadata."""
        if not isinstance(pin_info, dict):
            return None
        rotation = str(pin_info.get('rotation') or pin_info.get('rot') or '').strip().upper()
        if not rotation:
            return None
        normalized = rotation
        while normalized and normalized[0] in {'M', 'S'}:
            normalized = normalized[1:]
        if not normalized.startswith('R'):
            return None
        digits = ''.join(ch for ch in normalized[1:] if ch.isdigit())
        if not digits:
            return None
        angle = int(digits) % 360
        if angle in (0, 180):
            direction = 1 if angle == 0 else -1
            return 'x', direction
        if angle in (90, 270):
            direction = 1 if angle == 90 else -1
            return 'y', direction
        return None

    def _limit_vertical_keepout(self, pin_info: Optional[Dict], default_distance: float) -> float:
        """Clamp the vertical keep-out distance based on adjacent pin spacing."""
        min_vertical = max(self.grid_step * 0.15, 0.381)
        pitch = 0.0
        if isinstance(pin_info, dict):
            try:
                pitch = abs(float(pin_info.get('vertical_pitch') or 0.0))
            except (TypeError, ValueError):
                pitch = 0.0
        if pitch > 0.0:
            safe_limit = max(min_vertical, pitch * 0.45)
            return max(min_vertical, min(default_distance, safe_limit))
        return max(min_vertical, min(default_distance, self.grid_step))

    def _apply_pin_keepout(self, segment: ET.Element, pin_x: float, pin_y: float, pin_info: Dict,
                           net_id: Optional[str] = None) -> Tuple[float, float]:
        """Move the wire origin away from the pin to leave a keep-out gap."""
        if pin_info is None:
            return pin_x, pin_y

        keepout = self._pin_keepout_distance()
        if keepout <= 0:
            return pin_x, pin_y

        rel_x = float(pin_info.get('x', 0.0)) if isinstance(pin_info, dict) else 0.0
        rel_y = float(pin_info.get('y', 0.0)) if isinstance(pin_info, dict) else 0.0

        axis_direction = self._derive_pin_orientation(pin_info)
        if axis_direction:
            axis, direction = axis_direction
        else:
            axis = 'x'
            direction = 1
            if abs(rel_x) >= abs(rel_y) and abs(rel_x) > 0.01:
                direction = 1 if rel_x >= 0 else -1
            elif abs(rel_y) > 0.01:
                axis = 'y'
                direction = 1 if rel_y >= 0 else -1
            else:
                axis = 'y'
                direction = 1

        axis_keepout = keepout
        if axis == 'y':
            axis_keepout = self._limit_vertical_keepout(pin_info, keepout)

        if axis == 'x':
            exit_x = self._snap_to_grid(pin_x + direction * axis_keepout)
            exit_y = pin_y
        else:
            exit_x = pin_x
            exit_y = self._snap_to_grid(pin_y + direction * axis_keepout)

        if abs(exit_x - pin_x) > 0.01 or abs(exit_y - pin_y) > 0.01:
            ET.SubElement(segment, 'wire',
                          x1=str(pin_x), y1=str(pin_y),
                          x2=str(exit_x), y2=str(exit_y),
                          width="0.1524", layer="91")
            # Register segment with wire registry
            if net_id:
                self.wire_registry.register_segment(net_id, pin_x, pin_y, exit_x, exit_y)

        return exit_x, exit_y

    def _route_vertical_leg(self, segment: ET.Element, pin_x: float, pin_y: float, lane_y: float,
                            pin_info: Dict, component_ref: Optional[str] = None, net_id: Optional[str] = None) -> float:
        """Route from a pin to the horizontal lane and return the lane entry X coordinate."""
        effective_net_id = net_id or f"net_{component_ref or 'unknown'}"
        
        pin_x, pin_y = self._apply_pin_keepout(segment, pin_x, pin_y, pin_info, net_id=effective_net_id)
        axis_direction = self._derive_pin_orientation(pin_info)
        current_x = self._apply_horizontal_fanout(segment, component_ref, pin_info, pin_x, pin_y, axis_direction, net_id=effective_net_id)
        current_y = pin_y
        if abs(lane_y - current_y) < 0.01:
            return self._snap_to_grid(current_x)

        direction = 1.0 if lane_y > current_y else -1.0
        stage_y = self._compute_departure_y(current_y, lane_y, direction)

        vertical_start_y = stage_y
        entry_x = self._reserve_vertical_path(current_x, vertical_start_y, lane_y, target_lane_y=lane_y, 
                                              component_ref=component_ref, net_id=effective_net_id)

        if abs(stage_y - current_y) > 0.01:
            ET.SubElement(segment, 'wire',
                          x1=str(current_x), y1=str(current_y),
                          x2=str(current_x), y2=str(stage_y),
                          width="0.1524", layer="91")
            # Register segment with wire registry
            self.wire_registry.register_segment(effective_net_id, current_x, current_y, current_x, stage_y)
            current_y = stage_y

        if abs(entry_x - current_x) > 0.01:
            ET.SubElement(segment, 'wire',
                          x1=str(current_x), y1=str(current_y),
                          x2=str(entry_x), y2=str(current_y),
                          width="0.1524", layer="91")
            # Register segment with wire registry
            self.wire_registry.register_segment(effective_net_id, current_x, current_y, entry_x, current_y)
            current_x = entry_x

        if abs(lane_y - current_y) > 0.01:
            ET.SubElement(segment, 'wire',
                          x1=str(current_x), y1=str(current_y),
                          x2=str(current_x), y2=str(lane_y),
                          width="0.1524", layer="91")
            # Register segment with wire registry
            self.wire_registry.register_segment(effective_net_id, current_x, current_y, current_x, lane_y)

        return current_x
    
    def _add_nets(self, nets_elem, busses_elem: Optional[ET.Element] = None):
        """Add nets with wires connecting actual pin coordinates"""
        net_counter = 1
        supply_net_elements: Dict[str, ET.Element] = {}
        connections = self.data.get('connections') or []

        supply_pinrefs_seen: Dict[Tuple[str, str, str], str] = {}

        regular_connections: List[Dict] = []
        node_data: Dict[Tuple[str, str], Dict] = {}
        parent: Dict[Tuple[str, str], Tuple[str, str]] = {}

        def find(key: Tuple[str, str]) -> Tuple[str, str]:
            parent.setdefault(key, key)
            if parent[key] != key:
                parent[key] = find(parent[key])
            return parent[key]

        def union(a: Tuple[str, str], b: Tuple[str, str]):
            root_a = find(a)
            root_b = find(b)
            if root_a != root_b:
                parent[root_b] = root_a

        def ensure_node(key: Tuple[str, str], ref: str, pin_name: str,
                        pin_info: Dict, pin_x: float, pin_y: float):
            if key in node_data:
                return
            node_data[key] = {
                'ref': ref,
                'pin': pin_name,
                'pin_info': pin_info,
                'pin_x': pin_x,
                'pin_y': pin_y,
            }

        # PHASE 1: Collect all supply connection pins by net
        supply_pin_groups: Dict[str, List[Dict]] = {}
        
        for index, connection in enumerate(connections):
            from_entry = connection['from']
            to_entry = connection['to']
            from_ref = from_entry.get('ref')
            to_ref = to_entry.get('ref')
            from_is_supply = self._is_supply_ref(from_ref)
            to_is_supply = self._is_supply_ref(to_ref)

            if not (from_is_supply or to_is_supply):
                continue

            supply_entry = from_entry if from_is_supply else to_entry
            comp_entry = to_entry if from_is_supply else from_entry
            comp_side = 'to' if from_is_supply else 'from'
            supply_net_label = supply_entry.get('ref')

            if not supply_net_label:
                continue

            symbol_info = self._resolve_supply_symbol(supply_net_label)
            if not symbol_info:
                symbol_info = self._resolve_supply_symbol((supply_net_label or '').upper())
            if not symbol_info:
                continue

            net_name_attr = symbol_info['name'] or supply_net_label
            net_key = re.sub(r'\s+', '', net_name_attr.upper())
            comp_ref = comp_entry.get('ref')

            if not comp_ref:
                continue
            if comp_ref not in self.component_placements:
                self.pin_errors.append({
                    'ref': comp_ref,
                    'pin': comp_entry.get('pin', '?'),
                    'label': comp_entry.get('label', ''),
                    'value': self._get_component_value(comp_ref),
                    'available': [],
                    'library': None,
                    'deviceset': None,
                    'device': None,
                    'error': f"Component {comp_ref} not found in Eagle libraries — supply connection to {supply_net_label} dropped"
                })
                self.debug_messages.append(f"⚠ Supply connection skipped: {comp_ref} not placed (net {supply_net_label})")
                continue

            supply_pin_groups.setdefault(net_key, []).append({
                'index': index,
                'comp_side': comp_side,
                'comp_ref': comp_ref,
                'comp_pin': comp_entry.get('pin'),
                'comp_label': comp_entry.get('label'),
                'net_name': net_name_attr,
                'symbol_info': symbol_info
            })

        # PHASE 2: SIMPLE SUPPLY NETS - Each pin gets its own segment with just a pinref
        # NO WIRES between supply pins - Eagle connects them via the shared net name
        # Each component pin that connects to a supply gets a short wire to the supply symbol
        for net_key, pin_list in supply_pin_groups.items():
            if not pin_list:
                continue

            net_name_attr = pin_list[0]['net_name']
            symbol_info = pin_list[0]['symbol_info']
            supply_part_ref = symbol_info['part']
            supply_info = self.supply_details.get(supply_part_ref, {})
            has_symbol = supply_info.get('has_symbol', True)
            
            # Create the net
            net = ET.SubElement(nets_elem, 'net', name=net_name_attr, **{'class': '0'})
            supply_net_elements[net_key] = net
            
            supply_gate = supply_info.get('gate', 'G$1')
            supply_pin_name = supply_info.get('pin', net_name_attr)
            
            # Get supply symbol position
            supply_pin_x = None
            supply_pin_y = None
            if has_symbol and supply_part_ref in self.component_placements:
                supply_placement = self.component_placements[supply_part_ref]
                supply_offset_x = supply_info.get('offset_x', 0.0)
                supply_offset_y = supply_info.get('offset_y', 0.0)
                supply_pin_x = self._snap_to_grid(supply_placement['x'] + supply_offset_x)
                supply_pin_y = self._snap_to_grid(supply_placement['y'] + supply_offset_y)
            
            # Create ONE segment per component pin - each in its own isolated segment
            # This prevents "fallen apart" errors because each segment is self-contained
            for pin_dict in pin_list:
                comp_ref = pin_dict['comp_ref']
                comp_pin = pin_dict['comp_pin']
                comp_label = pin_dict['comp_label']

                comp_placement = self.component_placements.get(comp_ref)
                if not comp_placement:
                    continue
                    
                comp_pins = comp_placement.get('pins', {})
                comp_pin_resolved, comp_pin_info = self._resolve_pin(comp_ref, comp_pins, comp_pin, comp_label)

                if not (comp_pin_resolved and comp_pin_info):
                    self.pin_errors.append(self._build_pin_error(
                        comp_ref, comp_pin, comp_label,
                        f"Could not resolve pin for {comp_ref} pin {comp_pin}",
                        comp_pins))
                    continue

                comp_gate = comp_pin_info.get('gate', 'G$1')
                has_coords = 'x' in comp_pin_info and 'y' in comp_pin_info

                if has_coords:
                    comp_pin_x = self._snap_to_grid(comp_placement['x'] + comp_pin_info.get('gate_x', 0) + comp_pin_info['x'])
                    comp_pin_y = self._snap_to_grid(comp_placement['y'] + comp_pin_info.get('gate_y', 0) + comp_pin_info['y'])

                    segment = ET.SubElement(net, 'segment')
                    actual_pin = comp_pin_info.get('pin_name', comp_pin_resolved)
                    ET.SubElement(segment, 'pinref', part=comp_ref, gate=comp_gate, pin=actual_pin)

                    stub_length = self.grid_step * 1.0
                    pin_rotation = str(comp_pin_info.get('rotation', '')) if isinstance(comp_pin_info, dict) else ''

                    if 'R180' in pin_rotation:
                        stub_end_x = comp_pin_x + stub_length
                        stub_end_y = comp_pin_y
                    elif 'R90' in pin_rotation:
                        stub_end_x = comp_pin_x
                        stub_end_y = comp_pin_y + stub_length
                    elif 'R270' in pin_rotation:
                        stub_end_x = comp_pin_x
                        stub_end_y = comp_pin_y - stub_length
                    else:
                        stub_end_x = comp_pin_x - stub_length
                        stub_end_y = comp_pin_y

                    ET.SubElement(segment, 'wire',
                                 x1=str(comp_pin_x), y1=str(comp_pin_y),
                                 x2=str(stub_end_x), y2=str(stub_end_y),
                                 width="0.1524", layer="91")

                    ET.SubElement(segment, 'label',
                                 x=str(stub_end_x), y=str(stub_end_y),
                                 size="1.778", layer="95")
                else:
                    # Power pins in separate gates may lack coordinates;
                    # still add a pinref-only segment so Eagle connects via net name
                    segment = ET.SubElement(net, 'segment')
                    actual_pin = comp_pin_info.get('pin_name', comp_pin_resolved)
                    ET.SubElement(segment, 'pinref', part=comp_ref, gate=comp_gate, pin=actual_pin)
                    fallback_x = comp_placement['x']
                    fallback_y = comp_placement['y']
                    ET.SubElement(segment, 'wire',
                                 x1=str(fallback_x), y1=str(fallback_y),
                                 x2=str(fallback_x + self.grid_step), y2=str(fallback_y),
                                 width="0.1524", layer="91")

                self._record_connected_pin(pin_dict['index'], pin_dict['comp_side'], comp_ref,
                                          comp_pin, comp_pin_resolved, comp_label, net_name_attr)
                self._mark_supply_connected_pin(comp_ref, comp_pin_resolved, net_name_attr)
            
            # Add one segment for the supply symbol itself (if it exists)
            if has_symbol and supply_pin_x is not None and supply_pin_y is not None:
                segment = ET.SubElement(net, 'segment')
                ET.SubElement(segment, 'pinref', part=supply_part_ref, gate=supply_gate, pin=supply_pin_name)
                # Minimal wire stub - no snapping to avoid collisions
                stub_length = self.grid_step * 1.0
                stub_end_x = supply_pin_x + stub_length
                ET.SubElement(segment, 'wire',
                             x1=str(supply_pin_x), y1=str(supply_pin_y),
                             x2=str(stub_end_x), y2=str(supply_pin_y),
                             width="0.1524", layer="91")
                ET.SubElement(segment, 'label',
                             x=str(stub_end_x), y=str(supply_pin_y),
                             size="1.778", layer="95")

        # PHASE 3: Handle regular component-to-component connections
        skipped_supply = 0
        skipped_placement = 0
        skipped_resolve = 0
        processed_regular = 0
        for index, connection in enumerate(connections):
            from_entry = connection['from']
            to_entry = connection['to']
            from_ref = from_entry.get('ref')
            to_ref = to_entry.get('ref')
            from_pin = from_entry.get('pin')
            to_pin = to_entry.get('pin')
            from_is_supply = self._is_supply_ref(from_ref)
            to_is_supply = self._is_supply_ref(to_ref)

            # Skip supply connections (already handled)
            if from_is_supply or to_is_supply:
                skipped_supply += 1
                continue

            if from_ref not in self.component_placements or to_ref not in self.component_placements:
                missing_refs = []
                if from_ref not in self.component_placements:
                    missing_refs.append(from_ref)
                if to_ref not in self.component_placements:
                    missing_refs.append(to_ref)
                for mref in missing_refs:
                    mentry = from_entry if mref == from_ref else to_entry
                    mpin = mentry.get('pin', '?')
                    mlabel = mentry.get('label', '')
                    other_ref = to_ref if mref == from_ref else from_ref
                    other_pin = to_pin if mref == from_ref else from_pin
                    self.pin_errors.append({
                        'ref': mref,
                        'pin': mpin,
                        'label': mlabel,
                        'value': self._get_component_value(mref),
                        'available': [],
                        'library': None,
                        'deviceset': None,
                        'device': None,
                        'error': f"Component {mref} not found in Eagle libraries — connection to {other_ref}.Pin{other_pin} dropped"
                    })
                self.debug_messages.append(f"⚠ Connection skipped (no placement): {from_ref}.Pin{from_pin} -> {to_ref}.Pin{to_pin}  [{', '.join(missing_refs)} not placed]")
                skipped_placement += 1
                continue

            from_comp = self.component_placements[from_ref]
            to_comp = self.component_placements[to_ref]

            from_pins = from_comp.get('pins', {})
            to_pins = to_comp.get('pins', {})

            from_pin_resolved, from_pin_info = self._resolve_pin(from_ref, from_pins, from_pin, connection['from'].get('label'))
            to_pin_resolved, to_pin_info = self._resolve_pin(to_ref, to_pins, to_pin, connection['to'].get('label'))

            # Debug: log resolution for U6/J7 connections
            if from_ref in ('U6', 'J7') or to_ref in ('U6', 'J7'):
                self.debug_messages.append(
                    f"  PIN RESOLVE: {from_ref}.Pin{from_pin}(label={connection['from'].get('label')}) → '{from_pin_resolved}' | "
                    f"{to_ref}.Pin{to_pin}(label={connection['to'].get('label')}) → '{to_pin_resolved}'"
                )
                # One-time dump of J7 pin map
                if not hasattr(self, '_j7_pins_dumped'):
                    self._j7_pins_dumped = True
                    j7_placement = self.component_placements.get('J7', {})
                    j7_pins = j7_placement.get('pins', {})
                    self.debug_messages.append(f"  J7 PIN MAP ({len(j7_pins)} pins): {sorted(j7_pins.keys())}")
                    u6_placement = self.component_placements.get('U6', {})
                    u6_pins = u6_placement.get('pins', {})
                    self.debug_messages.append(f"  U6 PIN MAP ({len(u6_pins)} pins): {sorted(u6_pins.keys())}")

            if not (from_pin_resolved and from_pin_info and to_pin_resolved and to_pin_info):
                if not from_pin_resolved or not from_pin_info:
                    self.debug_messages.append(f"⚠ Pin resolve FAIL: {from_ref}.Pin{from_pin} (label={connection['from'].get('label')}) "
                          f"available={sorted(from_pins.keys())[:10]}")
                    self.pin_errors.append(self._build_pin_error(
                        from_ref, from_pin, connection['from'].get('label'),
                        f"Could not resolve pin for {from_ref} pin {from_pin}",
                        from_pins))
                if not to_pin_resolved or not to_pin_info:
                    self.debug_messages.append(f"⚠ Pin resolve FAIL: {to_ref}.Pin{to_pin} (label={connection['to'].get('label')}) "
                          f"available={sorted(to_pins.keys())[:10]}")
                    self.pin_errors.append(self._build_pin_error(
                        to_ref, to_pin, connection['to'].get('label'),
                        f"Could not resolve pin for {to_ref} pin {to_pin}",
                        to_pins))
                skipped_resolve += 1
                continue

            if 'x' not in from_pin_info or 'y' not in from_pin_info:
                self.pin_errors.append(self._build_pin_error(
                    from_ref, from_pin, connection['from'].get('label'),
                    f"Pin {from_pin_resolved} has no coordinates in symbol",
                    from_pins))
                continue

            if 'x' not in to_pin_info or 'y' not in to_pin_info:
                self.pin_errors.append(self._build_pin_error(
                    to_ref, to_pin, connection['to'].get('label'),
                    f"Pin {to_pin_resolved} has no coordinates in symbol",
                    to_pins))
                continue

            from_pin_x = self._snap_to_grid(from_comp['x'] + from_pin_info.get('gate_x', 0) + from_pin_info['x'])
            from_pin_y = self._snap_to_grid(from_comp['y'] + from_pin_info.get('gate_y', 0) + from_pin_info['y'])
            to_pin_x = self._snap_to_grid(to_comp['x'] + to_pin_info.get('gate_x', 0) + to_pin_info['x'])
            to_pin_y = self._snap_to_grid(to_comp['y'] + to_pin_info.get('gate_y', 0) + to_pin_info['y'])

            from_key = (from_ref, from_pin_resolved)
            to_key = (to_ref, to_pin_resolved)

            ensure_node(from_key, from_ref, from_pin_resolved, from_pin_info, from_pin_x, from_pin_y)
            ensure_node(to_key, to_ref, to_pin_resolved, to_pin_info, to_pin_x, to_pin_y)
            union(from_key, to_key)

            regular_connections.append({
                'conn_index': index,
                'from_key': from_key,
                'to_key': to_key,
                'from_ref': from_ref,
                'to_ref': to_ref,
                'from_requested_pin': from_pin,
                'to_requested_pin': to_pin,
                'from_label': connection['from'].get('label'),
                'to_label': connection['to'].get('label'),
                'from_resolved_pin': from_pin_resolved,
                'to_resolved_pin': to_pin_resolved,
                'net': connection.get('net')
            })
            processed_regular += 1

        self.debug_messages.append(f"\n=== Connection Processing Summary ===")
        self.debug_messages.append(f"Total connections: {len(connections)}")
        self.debug_messages.append(f"Supply connections: {skipped_supply}")
        self.debug_messages.append(f"Regular connections processed: {processed_regular}")
        self.debug_messages.append(f"Skipped (no placement): {skipped_placement}")
        self.debug_messages.append(f"Skipped (pin resolve fail): {skipped_resolve}")
        self.debug_messages.append(f"=====================================\n")

        # STUB-BASED ROUTING: Each pin gets its own segment with a short stub + label.
        # Eagle connects pins sharing the same net name.
        # CRITICAL: Use union-find roots to assign net names so that pins in the
        # same group share a net, but pins in DIFFERENT groups get DIFFERENT nets.
        processed_nets: Dict[str, ET.Element] = {}
        group_net_names: Dict[Tuple[str, str], str] = {}  # union-find root → net name
        processed_pins: Set[Tuple[str, str]] = set()

        for conn in regular_connections:
            from_key = conn['from_key']
            to_key = conn['to_key']

            if from_key not in node_data or to_key not in node_data:
                continue

            from_node = node_data[from_key]
            to_node = node_data[to_key]

            supply_only, supply_name = self._group_nodes_share_supply([from_key, to_key])
            if supply_only:
                assigned_name = supply_name or 'SUPPLY'
                self._record_connected_pin(
                    conn['conn_index'], 'from',
                    conn['from_ref'], conn['from_requested_pin'],
                    conn['from_resolved_pin'], conn['from_label'], assigned_name
                )
                self._record_connected_pin(
                    conn['conn_index'], 'to',
                    conn['to_ref'], conn['to_requested_pin'],
                    conn['to_resolved_pin'], conn['to_label'], assigned_name
                )
                continue

            # Use the union-find root to determine which group this connection belongs to
            group_root = find(from_key)
            if group_root in group_net_names:
                net_name = group_net_names[group_root]
            else:
                preferred_name = conn.get('net')
                net_name = self._format_net_name(preferred_name) if preferred_name else ''
                if not net_name:
                    net_name = f"N${net_counter}"
                    net_counter += 1
                group_net_names[group_root] = net_name

            if net_name not in processed_nets:
                net = ET.SubElement(nets_elem, 'net', name=net_name, **{'class': '0'})
                processed_nets[net_name] = net
            net = processed_nets[net_name]

            # Debug: log net assignment for U6/J7
            if from_node['ref'] in ('U6', 'J7') or to_node['ref'] in ('U6', 'J7'):
                self.debug_messages.append(
                    f"  NET ASSIGN: ({from_node['ref']},{from_node['pin']}) + ({to_node['ref']},{to_node['pin']}) → {net_name}"
                    f"  [root={group_root}]"
                )

            # Create a stub segment for each pin that hasn't been processed yet
            for node, side, req_pin, res_pin, label in [
                (from_node, 'from', conn['from_requested_pin'], conn['from_resolved_pin'], conn['from_label']),
                (to_node, 'to', conn['to_requested_pin'], conn['to_resolved_pin'], conn['to_label'])
            ]:
                pin_id = (node['ref'], node['pin'])
                if pin_id in processed_pins:
                    continue
                processed_pins.add(pin_id)

                segment = ET.SubElement(net, 'segment')

                pin_x = node['pin_x']
                pin_y = node['pin_y']
                gate = node['pin_info'].get('gate', 'G$1') if isinstance(node['pin_info'], dict) else 'G$1'

                actual_pin = node['pin_info'].get('pin_name', node['pin']) if isinstance(node['pin_info'], dict) else node['pin']
                ET.SubElement(segment, 'pinref', part=node['ref'], gate=gate, pin=actual_pin)

                stub_length = self.grid_step * 1.0
                pin_info = node['pin_info']
                pin_rotation = str(pin_info.get('rotation', '')) if isinstance(pin_info, dict) else ''

                if 'R180' in pin_rotation:
                    stub_end_x = pin_x + stub_length
                    stub_end_y = pin_y
                elif 'R90' in pin_rotation:
                    stub_end_x = pin_x
                    stub_end_y = pin_y + stub_length
                elif 'R270' in pin_rotation:
                    stub_end_x = pin_x
                    stub_end_y = pin_y - stub_length
                else:
                    stub_end_x = pin_x - stub_length
                    stub_end_y = pin_y

                ET.SubElement(segment, 'wire',
                             x1=str(pin_x), y1=str(pin_y),
                             x2=str(stub_end_x), y2=str(stub_end_y),
                             width="0.1524", layer="91")

                ET.SubElement(segment, 'label',
                             x=str(stub_end_x), y=str(stub_end_y),
                             size="1.778", layer="95")

            self._record_connected_pin(
                conn['conn_index'], 'from',
                conn['from_ref'], conn['from_requested_pin'],
                conn['from_resolved_pin'], conn['from_label'], net_name
            )
            self._record_connected_pin(
                conn['conn_index'], 'to',
                conn['to_ref'], conn['to_requested_pin'],
                conn['to_resolved_pin'], conn['to_label'], net_name
            )


        self._validate_connection_coverage()
        self._detect_unreferenced_power_pins()

    def _select_group_net_name(self, group_connections: Optional[List[Dict]]) -> Optional[str]:
        """Return the first meaningful net label from a set of unioned connections."""
        if not group_connections:
            return None
        for conn in group_connections:
            net_label = conn.get('net')
            if isinstance(net_label, str) and net_label.strip():
                return net_label
        return None

    def _resolve_connector_pin(self, ref: str, pin_map: Dict, requested_pin: str) -> Tuple[Optional[str], Optional[Dict]]:
        """Direct connector resolution: MD pin number → library pin by number/pad/position."""
        if not pin_map or not requested_pin or not requested_pin.isdigit():
            return None, None
        # Exact name match: "1" → "1"
        if requested_pin in pin_map:
            return requested_pin, pin_map[requested_pin]
        # Pad match: MD pin "1" → library pin with pad "1"
        for key, value in pin_map.items():
            pad = str(value.get('pad') or '').strip()
            if not pad:
                continue
            for pt in pad.split():
                if pt == requested_pin:
                    return key, value
        # P$N or -N style
        for variant in (f"P${requested_pin}", f"-{requested_pin}"):
            if variant in pin_map:
                return variant, pin_map[variant]
        # Positional: MD pin 1 = first pin, pin 2 = second, etc.
        idx = int(requested_pin)
        ordered = self._order_connector_pins(pin_map)
        if 1 <= idx <= len(ordered):
            key = ordered[idx - 1]
            return key, pin_map[key]
        # Single-pin connector: map any pin to the only available pin
        if len(pin_map) == 1:
            key = next(iter(pin_map))
            return key, pin_map[key]
        return None, None

    def _resolve_pin(self, ref: str, pin_map: Dict, requested_pin: str, pin_label: Optional[str]) -> Tuple[Optional[str], Optional[Dict]]:
        """Resolve pin name from requested pin number or label"""
        requested_pin = (requested_pin or '').strip()
        if not pin_map:
            return requested_pin or None, None

        overrides = self.pin_overrides.get(ref, {})
        for cand in [c for c in [requested_pin, (pin_label or '').strip()] if c]:
            override_target = overrides.get(cand)
            if override_target:
                if override_target in pin_map:
                    return override_target, pin_map[override_target]
                for key in pin_map:
                    if key.upper() == override_target.upper():
                        return key, pin_map[key]

        # Connectors: direct number-to-number matching
        if ref.startswith('J'):
            result = self._resolve_connector_pin(ref, pin_map, requested_pin)
            if result[0] is not None:
                return result

        # Passives (R, C, L): MD Pin 1/2 must map to distinct physical pins by position.
        # Some Eagle libraries use "pas 1"/"pas 2" or "~" for both; pad-based ordering
        # ensures we never short both ends to the same junction.
        if ref.startswith(('R', 'C', 'L')) and requested_pin and requested_pin.isdigit():
            idx = int(requested_pin)
            ordered = self._order_connector_pins(pin_map)
            if 1 <= idx <= len(ordered):
                key = ordered[idx - 1]
                return key, pin_map[key]

        # Prioritize exact pin_label match (e.g. 1Y, 2Y, VCC)
        if pin_label:
            label_clean = pin_label.strip()
            if label_clean in pin_map:
                return label_clean, pin_map[label_clean]
            for k in pin_map:
                if k.upper() == label_clean.upper():
                    return k, pin_map[k]
            # Tilde-aware: Eagle uses ~ for active-low (2~A vs 2A)
            label_no_tilde = label_clean.replace('~', '').upper()
            for k in pin_map:
                if k.replace('~', '').upper() == label_no_tilde:
                    return k, pin_map[k]

        if ref.startswith(('D', 'LED')) and requested_pin in ('1', '2'):
            if requested_pin == '1' and 'A' in pin_map:
                return 'A', pin_map['A']
            if requested_pin == '2':
                if 'C' in pin_map:
                    return 'C', pin_map['C']
                if 'K' in pin_map:
                    return 'K', pin_map['K']

        # ICs/transistors: match MD pin number to Eagle pad (physical pin) first
        if ref.startswith(('U', 'Q')) and requested_pin and requested_pin.isdigit():
            for key, value in pin_map.items():
                pad = str(value.get('pad') or '').strip()
                if not pad:
                    continue
                pad_tokens = pad.split()
                for pt in pad_tokens:
                    if pt == requested_pin or (pt.isdigit() and (pt.lstrip('0') or '0') == (requested_pin.lstrip('0') or '0')):
                        return key, value

        candidates: List[str] = []
        if requested_pin:
            candidates.append(requested_pin)
        if pin_label:
            label_clean = pin_label.strip()
            if label_clean and label_clean not in candidates:
                candidates.append(label_clean)
            label_upper = label_clean.upper().replace(' ', '') if label_clean else ''
            if label_upper in ('+12V', '12V', 'VCC12', 'VCC5', '5V', '3V3'):
                for alias in ('VCC', 'VIN', 'V+', 'INPUT'):
                    if alias not in candidates:
                        candidates.append(alias)
            elif label_upper in ('GND', 'AGND', 'DGND', 'PGND'):
                for alias in ('GND', 'VSS', 'GROUND'):
                    if alias not in candidates:
                        candidates.append(alias)

        for cand in list(dict.fromkeys(candidates)):
            if cand in pin_map:
                return cand, pin_map[cand]

        for cand in list(dict.fromkeys(candidates)):
            upper = cand.upper()
            for key, value in pin_map.items():
                if key.upper() == upper:
                    return key, value

        digit_candidates = [c for c in candidates if c.isdigit()]
        if digit_candidates:
            normalized = {d: d.lstrip('0') or '0' for d in digit_candidates}
            for key, value in pin_map.items():
                pad = str(value.get('pad') or '').strip()
                if not pad:
                    continue
                pad_tokens = pad.split()
                pad_norm = pad.lstrip('0') or pad
                for cand, norm in normalized.items():
                    if pad == cand or pad_norm == norm:
                        return key, value
                    for pt in pad_tokens:
                        if pt == cand or pt == f"P${cand}":
                            return key, value
            for cand in digit_candidates:
                p_dollar = f"P${cand}"
                if p_dollar in pin_map:
                    return p_dollar, pin_map[p_dollar]

        # ICs with descriptive names (1CLR, 2D): match "1" to pins starting with "1"
        if requested_pin and requested_pin.isdigit():
            prefix_matches = [k for k in pin_map if k.startswith(requested_pin) and
                             (len(k) == len(requested_pin) or not k[len(requested_pin):].isdigit())]
            if len(prefix_matches) == 1:
                return prefix_matches[0], pin_map[prefix_matches[0]]
            if prefix_matches and requested_pin in ('1', '2', '3', '4', '5', '6', '7', '8', '9'):
                # Prefer pin that matches pin_label (e.g. 1Y when label is "1Y")
                if pin_label:
                    label_clean = pin_label.strip()
                    for key in prefix_matches:
                        if key == label_clean or key.upper() == label_clean.upper():
                            return key, pin_map[key]
                # Multiple matches (e.g. 1CLR, 1D, 1CLK) - pick by pad if available
                for key in sorted(prefix_matches):
                    pad = str(pin_map.get(key, {}).get('pad') or '').strip()
                    if pad == requested_pin or (pad and pad.split()[0] == requested_pin):
                        return key, pin_map[key]
                return prefix_matches[0], pin_map[prefix_matches[0]]

        # Connectors (J): MD Pin 1 = first physical pin, Pin 2 = second, etc.
        if ref.startswith('J') and requested_pin and requested_pin.isdigit():
            idx = int(requested_pin)
            ordered = self._order_connector_pins(pin_map)
            if 1 <= idx <= len(ordered):
                key = ordered[idx - 1]
                return key, pin_map[key]

        # Pin labels with / or - (e.g. 1REXT/CEXT): normalize and match
        if pin_label:
            norm_label = re.sub(r'[/\s\-]+', '', pin_label.upper())
            for key in pin_map:
                norm_key = re.sub(r'[/\s\-]+', '', key.upper())
                if norm_label == norm_key:
                    return key, pin_map[key]
                if len(norm_label) >= 3 and (norm_key.startswith(norm_label) or norm_label.startswith(norm_key)):
                    return key, pin_map[key]

        return None, None

    def _order_connector_pins(self, pin_map: Dict) -> List[str]:
        """Return pin keys sorted by physical position (1, 2, 3...). Used for connectors and passives."""
        def sort_key(item):
            k, v = item
            pad = str(v.get('pad') or '').strip().split()[0] if v.get('pad') else ''
            if pad and pad.isdigit():
                return (0, int(pad))
            if k.startswith('P$') and len(k) > 2 and k[2:].isdigit():
                return (0, int(k[2:]))
            if k.isdigit():
                return (0, int(k))
            if k.startswith('-') and len(k) > 1 and k[1:].isdigit():
                return (0, int(k[1:]))
            m = re.search(r'(\d+)\s*$', str(k))
            if m:
                return (0, int(m.group(1)))
            return (1, k)
        return [k for k, _ in sorted(pin_map.items(), key=sort_key)]


class EagleSchematicGeneratorGUI:
    """Main GUI application"""
    
    def __init__(self, root):
        self.root = root
        self.root.title(f"Eagle CAD Schematic Generator v{__version__}")
        self.root.geometry("900x700")
        
        self.md_file = tk.StringVar()
        self.eagle_dir = tk.StringVar()
        self.output_file = tk.StringVar()
        self._config_path = CONFIG_FILE
        self._suspend_config_updates = True
        self._load_config()
        self._suspend_config_updates = False
        self.md_file.trace_add('write', self._on_config_var_change)
        self.eagle_dir.trace_add('write', self._on_config_var_change)
        
        self.library_parser = None
        self.schematic_data = None
        self.pin_overrides: Dict[str, Dict[str, str]] = {}
        self.pin_override_path: Optional[Path] = None
        self._pin_mapping_window = None
        
        self._create_widgets()
        self._save_config()
    
    def _create_widgets(self):
        """Create GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Title
        title = ttk.Label(main_frame, text=f"Eagle CAD Schematic Generator v{__version__}",
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=3, pady=10)
        
        # MD File selection
        ttk.Label(main_frame, text="MD Schematic File:").grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Entry(main_frame, textvariable=self.md_file, width=50).grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5)
        ttk.Button(main_frame, text="Browse", command=self._browse_md).grid(row=1, column=2, pady=5, padx=5)
        
        # Eagle directory selection
        ttk.Label(main_frame, text="Eagle CAD 7.4 Directory:").grid(row=2, column=0, sticky=tk.W, pady=5)
        ttk.Entry(main_frame, textvariable=self.eagle_dir, width=50).grid(row=2, column=1, sticky=(tk.W, tk.E), pady=5)
        ttk.Button(main_frame, text="Browse", command=self._browse_eagle_dir).grid(row=2, column=2, pady=5, padx=5)
        
        # Buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=3, pady=20)
        
        ttk.Button(button_frame, text="1. Scan Eagle Libraries", 
                  command=self._scan_libraries, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="2. Parse MD File", 
                  command=self._parse_md, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="3. Generate SCH", 
                  command=self._generate_sch, width=25).pack(side=tk.LEFT, padx=5)
        
        # Second button row for mappings
        button_frame2 = ttk.Frame(main_frame)
        button_frame2.grid(row=4, column=0, columnspan=3, pady=5)
        
        ttk.Button(button_frame2, text="Auto-Map Components", 
                  command=self._auto_map_components, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame2, text="Edit Component Mappings", 
                  command=self._edit_mappings, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame2, text="Load Mappings", 
                  command=self._load_mappings, width=25).pack(side=tk.LEFT, padx=5)
        
        # Third button row
        button_frame3 = ttk.Frame(main_frame)
        button_frame3.grid(row=5, column=0, columnspan=3, pady=5)
        
        ttk.Button(button_frame3, text="Save Mappings", 
                  command=self._save_mappings, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame3, text="Edit Pin Overrides", 
                  command=self._edit_pin_overrides, width=25).pack(side=tk.LEFT, padx=5)
        
        # Progress bar
        self.progress = ttk.Progressbar(main_frame, mode='determinate')
        self.progress.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Ready", foreground="green")
        self.status_label.grid(row=8, column=0, columnspan=3, pady=5)
        
        # Log text area
        ttk.Label(main_frame, text="Log:").grid(row=9, column=0, sticky=tk.W, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(main_frame, height=20, width=80)
        self.log_text.grid(row=10, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        main_frame.rowconfigure(10, weight=1)

    def _on_config_var_change(self, *_):
        """Persist config when MD or Eagle directory fields change"""
        self._save_config()
    
    def _load_config(self):
        """Load saved config values for MD file and Eagle directory"""
        if not self._config_path.exists():
            return
        try:
            with self._config_path.open('r', encoding='utf-8') as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            return
        md_path = data.get('md_file', '')
        eagle_dir = data.get('eagle_dir', '')
        if md_path:
            self.md_file.set(md_path)
            if not self.output_file.get():
                try:
                    self.output_file.set(str(Path(md_path).with_suffix('.scr')))
                except Exception:
                    pass
        if eagle_dir:
            self.eagle_dir.set(eagle_dir)
    
    def _save_config(self):
        """Save current MD and Eagle directory selections to config file"""
        if self._suspend_config_updates:
            return
        data = {
            'md_file': self.md_file.get(),
            'eagle_dir': self.eagle_dir.get()
        }
        try:
            with self._config_path.open('w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except OSError as e:
            print(f"Warning: Unable to save config: {e}")
    
    def _setup_pin_override_storage(self):
        md_path_str = self.md_file.get().strip()
        if not md_path_str:
            self.pin_override_path = None
            self.pin_overrides = {}
            return
        md_path = Path(md_path_str)
        override_name = md_path.stem + "_pin_overrides.json"
        self.pin_override_path = md_path.with_name(override_name)
        self._log(f"Pin override file set to {self.pin_override_path}")
    
    def _load_pin_overrides(self):
        self.pin_overrides = {}
        if not self.pin_override_path:
            return
        if not self.pin_override_path.exists():
            self._log(f"No pin override file found at {self.pin_override_path.name}; starting fresh overrides")
            return
        try:
            with self.pin_override_path.open('r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, dict):
                    self.pin_overrides = data
                    self._log(f"Loaded {sum(len(pins) for pins in data.values())} pin override(s) from {self.pin_override_path.name}")
                else:
                    self._log(f"Pin override file {self.pin_override_path.name} had unexpected format; ignoring")
        except (OSError, json.JSONDecodeError) as exc:
            self._log(f"Warning: Could not load pin overrides: {exc}")
    
    def _save_pin_overrides(self):
        if not self.pin_override_path:
            return
        try:
            if not self.pin_overrides:
                if self.pin_override_path.exists():
                    self.pin_override_path.unlink()
                    self._log(f"Removed empty pin override file {self.pin_override_path.name}")
                return
            with self.pin_override_path.open('w', encoding='utf-8') as f:
                json.dump(self.pin_overrides, f, indent=2)
            self._log(f"Saved pin overrides to {self.pin_override_path.name}")
        except OSError as exc:
            self._log(f"Error saving pin overrides: {exc}")
    
    def _browse_md(self):
        """Browse for MD file"""
        filename = filedialog.askopenfilename(
            title="Select MD Schematic File",
            filetypes=[("Markdown files", "*.md"), ("All files", "*.*")]
        )
        if filename:
            self.md_file.set(filename)
            # Auto-set output file
            if not self.output_file.get():
                output = str(Path(filename).with_suffix('.scr'))
                self.output_file.set(output)
    
    def _browse_eagle_dir(self):
        """Browse for Eagle directory"""
        dirname = filedialog.askdirectory(title="Select Eagle CAD 7.4 Directory")
        if dirname:
            self.eagle_dir.set(dirname)
    
    def _browse_output(self):
        """Browse for output SCR file"""
        filename = filedialog.asksaveasfilename(
            title="Save SCR File",
            defaultextension=".scr",
            filetypes=[("SCR files", "*.scr"), ("All files", "*.*")]
        )
        if filename:
            self.output_file.set(filename)
    
    def _log(self, message: str):
        """Add message to log"""
        print(message)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.root.update()
    
    def _scan_libraries(self):
        """Scan Eagle libraries"""
        eagle_dir = self.eagle_dir.get()
        if not eagle_dir:
            messagebox.showerror("Error", "Please select Eagle CAD directory")
            return
        
        if not os.path.isdir(eagle_dir):
            messagebox.showerror("Error", "Invalid Eagle CAD directory")
            return
        
        self.library_parser = EagleLibraryParser(eagle_dir)
        cache_loaded, cache_message = self.library_parser.load_cache(LIB_CACHE_FILE)
        if cache_loaded:
            self._log(f"Library cache hit: {cache_message}")
            self.status_label.config(text="Library cache loaded", foreground="green")
            self.progress['value'] = 100
            self._show_cache_loaded_dialog(cache_message)
            return
        else:
            self._log(f"Cache miss: {cache_message}")

        self._perform_library_scan()

    def _show_cache_loaded_dialog(self, cache_message: str):
        dialog = tk.Toplevel(self.root)
        dialog.title("Library Cache Loaded")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.resizable(False, False)
        ttk.Label(dialog,
                  text="Library cache loaded successfully.",
                  font=('Arial', 11, 'bold')).pack(padx=16, pady=(16, 4), anchor=tk.W)
        ttk.Label(dialog,
                  text=cache_message,
                  wraplength=360,
                  justify=tk.LEFT).pack(padx=16, pady=(0, 12), anchor=tk.W)
        ttk.Label(dialog,
                  text="Rescan if libraries changed since the cache was created.",
                  font=('Arial', 9, 'italic'),
                  wraplength=360,
                  justify=tk.LEFT).pack(padx=16, pady=(0, 12), anchor=tk.W)

        button_frame = ttk.Frame(dialog)
        button_frame.pack(fill=tk.X, padx=16, pady=(0, 16))

        def close_dialog():
            try:
                dialog.grab_release()
            except tk.TclError:
                pass
            dialog.destroy()

        def trigger_rescan():
            close_dialog()
            self._log("User requested full library rescan after cache load")
            self.status_label.config(text="Scanning libraries...", foreground="blue")
            self.progress['value'] = 0
            self.root.after(50, self._perform_library_scan)

        ttk.Button(button_frame, text="Rescan Libraries", command=trigger_rescan, width=18).pack(side=tk.LEFT)
        ttk.Button(button_frame, text="Close", command=close_dialog, width=12).pack(side=tk.RIGHT)
        dialog.protocol("WM_DELETE_WINDOW", close_dialog)

    def _perform_library_scan(self):
        self._log("Scanning Eagle libraries...")
        self.status_label.config(text="Scanning libraries...", foreground="blue")
        self.progress['value'] = 0

        def progress_callback(current, total, filename):
            progress = (current / total) * 100
            self.progress['value'] = progress
            if total and current % 10 == 0:
                self._log(f"Scanning: {filename} ({current}/{total})")

        try:
            num_devices = self.library_parser.scan_libraries(progress_callback)
            self._log(f"\nLibrary scan complete!")
            self._log(f"Found {num_devices} devices in Eagle libraries")
            saved, save_message = self.library_parser.save_cache(LIB_CACHE_FILE)
            self._log(save_message)
            self.status_label.config(text="Library scan complete", foreground="green")
            self.progress['value'] = 100
            messagebox.showinfo("Success", f"Scanned libraries successfully!\nFound {num_devices} devices")
        except Exception as e:
            self._log(f"\nError scanning libraries: {str(e)}")
            self.status_label.config(text="Error scanning libraries", foreground="red")
            messagebox.showerror("Error", f"Failed to scan libraries:\n{str(e)}")
    
    def _parse_md(self):
        """Parse MD schematic file"""
        md_file = self.md_file.get()
        if not md_file:
            messagebox.showerror("Error", "Please select MD file")
            return
        
        if not os.path.isfile(md_file):
            messagebox.showerror("Error", "Invalid MD file")
            return
        
        self._log("\nParsing MD schematic file...")
        self.status_label.config(text="Parsing MD file...", foreground="blue")
        
        try:
            parser = MDSchematicParser(md_file)
            self.schematic_data = parser.parse()
            self._setup_pin_override_storage()
            self._load_pin_overrides()
            
            self._log(f"\nParsing complete!")
            self._log(f"Found {len(self.schematic_data['components'])} components")
            self._log(f"Found {len(self.schematic_data['nets'])} nets")
            self._log(f"Found {len(self.schematic_data['connections'])} connections")
            
            if len(self.schematic_data['connections']) > 0:
                self._log("\nSample connections:")
                for conn in self.schematic_data['connections'][:10]:
                    self._log(f"  {conn['from']['ref']}.{conn['from']['pin']} → {conn['to']['ref']}.{conn['to']['pin']}")
                if len(self.schematic_data['connections']) > 10:
                    self._log(f"  ... and {len(self.schematic_data['connections']) - 10} more")
            else:
                self._log("\n⚠️ WARNING: No connections found!")
                self._log("   Nets will not be created in Eagle schematic")
            
            self._log("\nComponents:")
            for comp in self.schematic_data['components']:
                self._log(f"  {comp['reference']}: {comp['value']}")

            parse_warnings = self.schematic_data.get('warnings', [])
            if parse_warnings:
                self._log(f"\n⚠️ CROSS-REFERENCE WARNINGS ({len(parse_warnings)}):")
                for w in parse_warnings:
                    self._log(f"  {w}")
                self._log("  Fix these in the MD file before generating the schematic.")

            self.status_label.config(text="MD parsing complete", foreground="green")
            if parse_warnings:
                messagebox.showwarning("Parsed with Warnings",
                    f"Parsed MD file successfully, but found {len(parse_warnings)} "
                    f"cross-reference warning(s). Check the log for details.")
            else:
                messagebox.showinfo("Success", "Parsed MD file successfully!")
        
        except Exception as e:
            self._log(f"\nError parsing MD file: {str(e)}")
            self.status_label.config(text="Error parsing MD file", foreground="red")
            messagebox.showerror("Error", f"Failed to parse MD file:\n{str(e)}")
    
    def _generate_scr(self):
        """Generate Eagle SCR file"""
        if not self.library_parser:
            messagebox.showerror("Error", "Please scan Eagle libraries first")
            return
        
        if not self.schematic_data:
            messagebox.showerror("Error", "Please parse MD file first")
            return
        
        output_file = self.output_file.get()
        if not output_file:
            messagebox.showerror("Error", "Please specify output file")
            return
        
        self._log("\nGenerating Eagle SCR file...")
        self.status_label.config(text="Generating SCR...", foreground="blue")
        
        try:
            total_overrides = sum(len(v) for v in self.pin_overrides.values())
            self._log(f"Starting SCR generation with {total_overrides} pin override(s) in memory")
            generator = EagleSCRGenerator(self.schematic_data, self.library_parser, self.pin_overrides)
            success, message = generator.generate(output_file)
            actionable_errors = self._filter_pin_errors(generator.pin_errors)
            if actionable_errors:
                issue_count = len(actionable_errors)
                message += (f"\n⚠️ {issue_count} pin mapping issue(s) detected on non-passive components."
                            " Use the pop-up mapper to align AI pin labels with the Eagle symbol.")
            
            self._log(f"\n{message}")
            if generator.override_usage:
                self._log("\nPin overrides applied during generation:")
                for usage in generator.override_usage:
                    self._log(f"  {usage['ref']}: requested '{usage['requested']}' → '{usage['mapped_to']}'")
            
            # Show component matching details
            if success and generator.component_placements:
                self._log("\n=== Component Matching Details ===")
                for ref, placement in sorted(generator.component_placements.items()):
                    lib = placement['library']
                    dev = placement['deviceset']
                    pkg = placement.get('device', '')
                    self._log(f"{ref}: {lib} / {dev}{pkg}")
            
            if success:
                self.status_label.config(text="SCR generation complete", foreground="green")
                messagebox.showinfo("Success", f"Generated SCR file successfully!\n\n{message}")
            else:
                self.status_label.config(text="SCR generation failed", foreground="red")
                messagebox.showerror("Error", message)
            if success and actionable_errors:
                self._open_pin_mapping_dialog(actionable_errors)
        
        except Exception as e:
            self._log(f"\nError generating SCR: {str(e)}")
            import traceback
            self._log(traceback.format_exc())
            self.status_label.config(text="Error generating SCR", foreground="red")
            messagebox.showerror("Error", f"Failed to generate SCR:\n{str(e)}")
    
    def _generate_sch(self):
        """Generate an Eagle .sch XML file with embedded libraries and wire routing"""
        if not self.schematic_data:
            messagebox.showerror("Error", "Please parse MD file first")
            return
        
        if not self.library_parser:
            messagebox.showerror("Error", "Please scan libraries first")
            return
        
        # Get output filename
        from tkinter import filedialog
        output_file = filedialog.asksaveasfilename(
            title="Save SCH File",
            defaultextension=".sch",
            filetypes=[("Eagle Schematic", "*.sch"), ("All Files", "*.*")]
        )
        
        if not output_file:
            return
        
        try:
            self._log("\n=== Generating Eagle SCH File ===")
            self.status_label.config(text="Preparing SCH generation...", foreground="blue")
            self.progress['value'] = 0
            self.root.update()
            
            # Determine which libraries are actually used
            used_libraries = set()
            for component in self.schematic_data['components']:
                ref = component['reference']
                value = component['value']
                pkg = component.get('package', '')
                lib_component = self.library_parser.find_component(ref, value, pkg)
                if lib_component:
                    used_libraries.add(lib_component['library'])
            
            # Add supply1 if any supply symbols are used
            supply_detected = False
            if self.schematic_data.get('connections'):
                for connection in self.schematic_data['connections']:
                    from_ref = connection['from']['ref']
                    to_ref = connection['to']['ref']
                    # Check if either is a supply net
                    if from_ref in ['GND', 'VCC', 'VDD', 'VSS', 'AGND', 'DGND', 'IOGND'] or \
                       to_ref in ['GND', 'VCC', 'VDD', 'VSS', 'AGND', 'DGND', 'IOGND'] or \
                       (from_ref and from_ref.startswith('+')) or (to_ref and to_ref.startswith('+')):
                        supply_detected = True
                        used_libraries.add('supply1')
                        self._log(f"Supply connection detected: {from_ref} → {to_ref}, adding supply1 library")
                        break
            
            if supply_detected:
                self._log(f"supply1 library added to used_libraries set")
            self._log(f"Used libraries: {sorted(used_libraries)}")
            
            # Deep scan only the libraries we're using (extracts pin coordinates and caches XML)
            self._log(f"\nDeep scanning {len(used_libraries)} used libraries...")
            self.status_label.config(text=f"Deep scanning {len(used_libraries)} libraries...", foreground="blue")
            
            def deep_scan_progress(current, total, name):
                self.progress['value'] = (current / total) * 50  # Use first 50% of progress bar
                self.status_label.config(text=f"Deep scanning: {name}", foreground="blue")
                self.root.update()
            
            self.library_parser.deep_scan_used_libraries(used_libraries, deep_scan_progress)
            
            self._log("Deep scan complete. Generating schematic...")
            self.status_label.config(text="Generating SCH...", foreground="blue")
            self.progress['value'] = 50
            self.root.update()
            
            # Create SCH writer
            writer = EagleSchematicWriter(
                self.schematic_data,
                self.library_parser,
                self.pin_overrides
            )
            
            # Generate the file
            success, message = writer.generate(output_file)
            
            self.progress['value'] = 100

            # Show writer diagnostics
            if writer.debug_messages:
                self._log("\n=== Writer Diagnostics ===")
                for msg in writer.debug_messages:
                    self._log(msg)

            actionable_errors: List[Dict] = []

            # Log pin errors if any
            if writer.pin_errors:
                self._log("\n⚠️ Pin mapping issues detected:")
                for err in writer.pin_errors:
                    self._log(f"  {err['ref']}: {err['error']}")
                actionable_errors = self._filter_pin_errors(writer.pin_errors)

            self._log(f"\n{message}")
            
            if writer.override_usage:
                self._log("\nPin overrides applied during generation:")
                for usage in writer.override_usage:
                    self._log(f"  {usage['ref']}: requested '{usage['requested']}' → '{usage['mapped_to']}'")
            
            # Show component placement details
            if success and writer.component_placements:
                self._log("\n=== Component Placement Details ===")
                for ref, placement in sorted(writer.component_placements.items()):
                    lib = placement['library']
                    dev = placement['deviceset']
                    pkg = placement.get('device', '')
                    x = placement['x']
                    y = placement['y']
                    self._log(f"{ref}: {lib}/{dev}{pkg} @ ({x:.1f}, {y:.1f})")
            
            if success:
                self.status_label.config(text="SCH generation complete", foreground="green")
                summary_lines = [
                    "SCH file created successfully.",
                    f"Output: {output_file}",
                    f"Components matched: {len(writer.component_placements)}"
                ]
                if actionable_errors:
                    summary_lines.append(f"Pin issues to review: {len(actionable_errors)}")
                messagebox.showinfo("Success", "\n".join(summary_lines))
            else:
                self.status_label.config(text="SCH generation failed", foreground="red")
                messagebox.showerror("Error", message)

            actionable_errors = self._filter_pin_errors(writer.pin_errors)
            if success and actionable_errors:
                self._open_pin_mapping_dialog(actionable_errors)
        
        except Exception as e:
            self._log(f"\nError generating SCH: {str(e)}")
            import traceback
            self._log(traceback.format_exc())
            self.status_label.config(text="Error generating SCH", foreground="red")
            messagebox.showerror("Error", f"Failed to generate SCH:\n{str(e)}")
    
    def _auto_map_components(self):
        """Automatically map all components from MD file to Eagle libraries"""
        if not self.schematic_data:
            messagebox.showerror("Error", "Please parse MD file first")
            return
        
        if not self.library_parser:
            eagle_dir = self.eagle_dir.get().strip()
            if not eagle_dir:
                messagebox.showerror("Error", "Please select Eagle CAD directory")
                return
            self._log("No library scan active; attempting to load cache automatically...")
            parser = EagleLibraryParser(eagle_dir)
            cache_loaded, cache_message = parser.load_cache(LIB_CACHE_FILE)
            if cache_loaded:
                self._log(f"Library cache auto-loaded: {cache_message}")
                self.library_parser = parser
            else:
                self._log(f"Cache unavailable: {cache_message}")
                messagebox.showerror(
                    "Libraries Not Ready",
                    "Please click 'Scan Eagle Libraries' once so device data is available "
                    "(a cache was not found)."
                )
                return
        
        if self.library_parser.manual_mappings:
            proceed = messagebox.askyesno(
                "Overwrite Existing Mappings?",
                ("Auto-mapping will replace all current component mappings."
                 " Do you want to continue?"),
                icon='warning'
            )
            if not proceed:
                self._log("Auto-mapping cancelled by user")
                return
        self.library_parser.manual_mappings.clear()
        self._log("\n=== Auto-Mapping Components ===")
        self.status_label.config(text="Auto-mapping components...", foreground="blue")
        
        mapped_count = 0
        failed_components = []
        
        # Auto-map each component
        for component in self.schematic_data['components']:
            ref = component['reference']
            value = component['value']
            pkg = component.get('package', '')
            
            # Try to find matching component
            lib_component = self.library_parser.find_component(ref, value, pkg)
            
            if lib_component:
                library = lib_component['library']
                deviceset = lib_component['deviceset']
                device = lib_component['device']
                
                # Add to manual mappings
                self.library_parser.add_manual_mapping(ref, library, deviceset, device)
                
                self._log(f"✓ {ref:8s} → {library}/{deviceset}/{device}")
                mapped_count += 1
                
                # Warn if connector mapped to single-pin device
                if ref.startswith('J'):
                    if 'BANANA' in deviceset.upper() or 'TESTPOINT' in deviceset.upper():
                        self._log(f"  ⚠️ WARNING: {ref} is a connector but mapped to single-pin device!")
                        self._log(f"     This will cause connection failures. Please fix manually.")
                        failed_components.append(f"{ref} ({value}) - mapped to wrong device type")
            else:
                self._log(f"✗ {ref:8s} - No match found")
                failed_components.append(f"{ref} ({value})")
        
        self._log(f"\nAuto-mapping complete!")
        self._log(f"Mapped: {mapped_count}/{len(self.schematic_data['components'])} components")
        
        if failed_components:
            self._log(f"\nFailed to map {len(failed_components)} components:")
            for comp in failed_components:
                self._log(f"  - {comp}")
        
        # Auto-save to default file
        default_mapping_file = self.md_file.get().replace('.md', '_mappings.json')
        if default_mapping_file:
            success, message = self.library_parser.save_manual_mappings(default_mapping_file)
            if success:
                self._log(f"\n✓ Auto-saved mappings to: {default_mapping_file}")
            else:
                self._log(f"\n✗ Could not auto-save: {message}")
        
        self.status_label.config(text="Auto-mapping complete", foreground="green")
        
        # Show summary with option to edit
        summary = f"Auto-mapping complete!\n\n"
        summary += f"✓ Mapped: {mapped_count} components\n"
        if failed_components:
            summary += f"✗ Failed: {len(failed_components)} components\n\n"
            summary += "Failed components:\n" + "\n".join(f"  • {c}" for c in failed_components[:5])
            if len(failed_components) > 5:
                summary += f"\n  ... and {len(failed_components) - 5} more"
        
        summary += f"\n\nMappings auto-saved to:\n{default_mapping_file if default_mapping_file else 'N/A'}"
        summary += "\n\nClick 'Edit Component Mappings' to review/fix any issues."
        
        messagebox.showinfo("Auto-Mapping Complete", summary)
    
    def _edit_mappings(self):
        """Open dialog to edit component mappings with library browser"""
        if not self.schematic_data:
            messagebox.showerror("Error", "Please parse MD file first")
            return
        
        if not self.library_parser:
            messagebox.showerror("Error", "Please scan libraries first")
            return
        
        # Create mapping editor window
        editor = tk.Toplevel(self.root)
        editor.title("Component Mapping Editor - Point & Click")
        editor.geometry("1260x780")
        
        # Create main paned window (split view)
        paned = ttk.PanedWindow(editor, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # LEFT PANEL: Component List
        left_frame = ttk.Frame(paned)
        paned.add(left_frame, weight=1)
        
        ttk.Label(left_frame, text="Components", font=('Arial', 12, 'bold')).pack(pady=5)
        ttk.Label(left_frame, text="Click a component to map it", font=('Arial', 9)).pack()
        
        # Legend frame
        legend_frame = ttk.Frame(left_frame)
        legend_frame.pack(fill=tk.X, padx=5, pady=5)
        
        legend_label = ttk.Label(legend_frame, text="Legend:", font=('Arial', 8, 'bold'))
        legend_label.grid(row=0, column=0, sticky=tk.W)
        
        white_box = tk.Label(legend_frame, text=" ", bg='white', relief=tk.RIDGE, width=2)
        white_box.grid(row=0, column=1, padx=2)
        ttk.Label(legend_frame, text="Not mapped", font=('Arial', 8)).grid(row=0, column=2, sticky=tk.W, padx=2)
        
        blue_box = tk.Label(legend_frame, text=" ", bg='lightblue', relief=tk.RIDGE, width=2)
        blue_box.grid(row=0, column=3, padx=2)
        ttk.Label(legend_frame, text="Auto-mapped", font=('Arial', 8)).grid(row=0, column=4, sticky=tk.W, padx=2)
        
        green_box = tk.Label(legend_frame, text=" ", bg='lightgreen', relief=tk.RIDGE, width=2)
        green_box.grid(row=0, column=5, padx=2)
        ttk.Label(legend_frame, text="Manually set", font=('Arial', 8)).grid(row=0, column=6, sticky=tk.W, padx=2)
        
        # Component listbox with scrollbar
        comp_list_frame = ttk.Frame(left_frame)
        comp_list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        comp_scrollbar = ttk.Scrollbar(comp_list_frame)
        comp_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        comp_listbox = tk.Listbox(comp_list_frame, yscrollcommand=comp_scrollbar.set,
                                   font=('Courier', 10), height=25)
        comp_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        comp_scrollbar.config(command=comp_listbox.yview)
        
        # Populate component list with mapping info
        comp_data = {}
        for component in self.schematic_data['components']:
            ref = component['reference']
            value = component['value']
            
            # Build display string with mapping info if available
            if self.library_parser and ref in self.library_parser.manual_mappings:
                mapping = self.library_parser.manual_mappings[ref]
                lib = mapping.get('library', '')
                dev = mapping.get('deviceset', '')
                display = f"{ref:8s} → {lib}/{dev}"
            else:
                display = f"{ref:8s} - {value[:40]}"
            
            comp_listbox.insert(tk.END, display)
            comp_data[ref] = component
        
        # Current selection display with mapping history
        selection_frame = ttk.LabelFrame(left_frame, text="Current Selection & Mapping", padding=10)
        selection_frame.pack(fill=tk.X, padx=5, pady=5)
        
        selected_ref_var = tk.StringVar(value="None")
        
        # Original auto-mapping
        ttk.Label(selection_frame, text="Component:", font=('Arial', 9, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(selection_frame, textvariable=selected_ref_var, font=('Arial', 9)).grid(row=0, column=1, sticky=tk.W, pady=2)
        
        ttk.Separator(selection_frame, orient='horizontal').grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Original/Auto mapping
        ttk.Label(selection_frame, text="Auto-Mapped To:", font=('Arial', 9, 'bold')).grid(row=2, column=0, sticky=tk.W)
        auto_mapping_var = tk.StringVar(value="")
        ttk.Label(selection_frame, textvariable=auto_mapping_var, font=('Arial', 8), 
                 foreground='blue', wraplength=200).grid(row=3, column=0, columnspan=2, sticky=tk.W, padx=10)
        
        ttk.Separator(selection_frame, orient='horizontal').grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Current/Changed mapping
        ttk.Label(selection_frame, text="Currently Mapped To:", font=('Arial', 9, 'bold')).grid(row=5, column=0, sticky=tk.W)
        
        selected_lib_var = tk.StringVar(value="")
        selected_dev_var = tk.StringVar(value="")
        selected_device_var = tk.StringVar(value="")
        
        current_mapping_text = tk.StringVar(value="")
        current_mapping_label = ttk.Label(selection_frame, textvariable=current_mapping_text, 
                                         font=('Arial', 8), foreground='green', wraplength=200)
        current_mapping_label.grid(row=6, column=0, columnspan=2, sticky=tk.W, padx=10)
        
        def update_mapping_display(ref):
            """Update the mapping display to show auto and current mappings"""
            # Get auto-mapping (from library parser if it exists)
            auto_map = ""
            if self.library_parser and ref in self.library_parser.manual_mappings:
                orig = self.library_parser.manual_mappings[ref]
                auto_map = f"{orig.get('library', '')}/{orig.get('deviceset', '')}/{orig.get('device', '')}"
            
            auto_mapping_var.set(auto_map if auto_map else "Not auto-mapped")
            
            # Get current mapping
            if ref in current_mappings:
                curr = current_mappings[ref]
                curr_map = f"{curr.get('library', '')}/{curr.get('deviceset', '')}/{curr.get('device', '')}"
                current_mapping_text.set(curr_map)
                
                # Highlight if changed
                if auto_map and auto_map != curr_map:
                    current_mapping_label.config(foreground='darkgreen', font=('Arial', 8, 'bold'))
                else:
                    current_mapping_label.config(foreground='blue', font=('Arial', 8))
            else:
                current_mapping_text.set("Not mapped yet")
                current_mapping_label.config(foreground='gray', font=('Arial', 8))
        
        # RIGHT PANEL: Library Browser
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=2)
        
        ttk.Label(right_frame, text="Library Browser", font=('Arial', 12, 'bold')).pack(pady=5)
        
        # Search box
        search_frame = ttk.Frame(right_frame)
        search_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(search_frame, text="Search:", font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=5)
        search_var = tk.StringVar()
        search_entry = ttk.Entry(search_frame, textvariable=search_var, width=30)
        search_entry.pack(side=tk.LEFT, padx=5)
        
        def do_search():
            search_term = search_var.get().upper()
            device_listbox.delete(0, tk.END)
            
            if not search_term:
                device_listbox.insert(tk.END, "Enter search term and click Search")
                return
            
            matches = []
            for device in self.library_parser._iter_unique_devices():
                deviceset = device.get('deviceset', '')
                library = device.get('library', '')
                desc = device.get('description', '')
                variant = device.get('device', '')
                package = device.get('package', '')

                if (search_term in deviceset.upper() or
                        search_term in library.upper() or
                        search_term in desc.upper() or
                        search_term in variant.upper() or
                        search_term in package.upper()):
                    matches.append(device)
            
            if matches:
                max_results = 150
                device_listbox.insert(tk.END, f"Found {len(matches)} matches (showing first {min(len(matches), max_results)}):")
                device_listbox.insert(tk.END, "")
                for device in matches[:max_results]:
                    lib = device.get('library', '')
                    dev = device.get('deviceset', '')
                    variant = device.get('device', '')
                    pkg = device.get('package', '')
                    display = f"{lib:20s} | {dev:25s} | {variant:15s} | {pkg}"
                    device_listbox.insert(tk.END, display)
            else:
                device_listbox.insert(tk.END, "No matches found")
        
        ttk.Button(search_frame, text="Search", command=do_search).pack(side=tk.LEFT, padx=5)
        ttk.Button(search_frame, text="Clear", command=lambda: [search_var.set(''), device_listbox.delete(0, tk.END)]).pack(side=tk.LEFT)
        
        # Device listbox with scrollbar
        device_list_frame = ttk.Frame(right_frame)
        device_list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        device_scrollbar = ttk.Scrollbar(device_list_frame)
        device_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        device_listbox = tk.Listbox(device_list_frame, yscrollcommand=device_scrollbar.set,
                                     font=('Courier', 9), height=25)
        device_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        device_scrollbar.config(command=device_listbox.yview)
        
        device_listbox.insert(tk.END, "Enter search term above to find components")
        device_listbox.insert(tk.END, "Examples: AD2428, PCM5100, OPAMP, R0805, LED")
        
        # Store mappings
        current_mappings = {}
        
        # Pre-load existing mappings
        for ref in comp_data.keys():
            if ref in self.library_parser.manual_mappings:
                current_mappings[ref] = self.library_parser.manual_mappings[ref].copy()
        
        # Selection handlers
        current_component = {'ref': None}
        
        def on_component_select(event):
            selection = comp_listbox.curselection()
            if not selection:
                return
            
            idx = selection[0]
            ref = list(comp_data.keys())[idx]
            current_component['ref'] = ref
            
            component_info = comp_data[ref]
            selected_ref_var.set(f"{ref} - {component_info['value'][:30]}")
            
            # Update mapping display
            update_mapping_display(ref)
            
            # Auto-search for this component
            value = comp_data[ref]['value']
            # Extract key terms from value for search
            search_term = ''
            
            # For ICs, try to extract part number
            if ref.startswith('U'):
                # Try patterns like AD2428, PCM5100A, AMS1117, 24LC
                ic_patterns = [r'(AD\d+)', r'(PCM\d+)', r'(AMS\d+)', r'(24LC\d+)', r'([A-Z]{2,}\d+)']
                for pattern in ic_patterns:
                    match = re.search(pattern, value.upper())
                    if match:
                        search_term = match.group(1)
                        break
            
            # For passives, use package
            elif ref.startswith(('R', 'C', 'L')):
                if '0805' in value:
                    search_term = f"{ref[0]}0805"
                elif '0603' in value:
                    search_term = f"{ref[0]}0603"
                elif '1206' in value:
                    search_term = f"{ref[0]}1206"
                else:
                    search_term = f"{ref[0]}0805"
            
            # For LEDs
            elif ref.startswith('LED'):
                search_term = 'LED'
            
            # For diodes
            elif ref.startswith('D'):
                if 'SCHOTTKY' in value.upper() or 'SS14' in value.upper():
                    search_term = 'SCHOTTKY'
                else:
                    search_term = 'DIODE'
            
            # For connectors
            elif ref.startswith('J'):
                if 'MOLEX' in value.upper():
                    molex_match = re.search(r'(\d{3,}[-\d]+)', value)
                    if molex_match:
                        search_term = molex_match.group(1)
                    else:
                        search_term = 'MOLEX'
                elif 'RCA' in value.upper():
                    search_term = 'RCA'
                elif 'BARREL' in value.upper() or 'JACK' in value.upper():
                    search_term = 'BARREL'
                elif 'HEADER' in value.upper() or 'PIN' in value.upper():
                    search_term = 'PINHD'
            
            # For crystals
            elif ref.startswith('Y'):
                search_term = 'CRYSTAL'
            
            if search_term:
                search_var.set(search_term)
                do_search()
        
        def on_device_select(event):
            selection = device_listbox.curselection()
            if not selection or not current_component['ref']:
                return
            
            idx = selection[0]
            line = device_listbox.get(idx)
            
            # Skip header/info lines
            if '|' not in line:
                return
            
            # Parse line: "library | deviceset | device | package"
            parts = [p.strip() for p in line.split('|')]
            if len(parts) >= 3:
                library = parts[0].strip()
                deviceset = parts[1].strip()
                device = parts[2].strip() if len(parts) > 2 else ''
                
                ref = current_component['ref']
                current_mappings[ref] = {
                    'library': library,
                    'deviceset': deviceset,
                    'device': device
                }
                
                # Update the display
                selected_lib_var.set(library)
                selected_dev_var.set(deviceset)
                selected_device_var.set(device)
                
                # Mark as manually changed
                manually_changed.add(ref)
                
                # Find the component index and update its color to GREEN (manual)
                comp_refs = list(comp_data.keys())
                if ref in comp_refs:
                    comp_idx = comp_refs.index(ref)
                    comp_listbox.itemconfig(comp_idx, bg='lightgreen')
                    
                    # Update the component list display to show the new mapping
                    new_display = f"{ref:8s} → {library}/{deviceset}"
                    comp_listbox.delete(comp_idx)
                    comp_listbox.insert(comp_idx, new_display)
                    comp_listbox.itemconfig(comp_idx, bg='lightgreen')
                    comp_listbox.selection_set(comp_idx)
                    
                    update_status()
                    update_mapping_display(ref)
                    
                    self._log(f"✓ Mapped {ref} → {library}/{deviceset}/{device}")
                    
                    # Validate pin count if we have connection info
                    self._validate_pin_count(ref, library, deviceset, device)
        
        comp_listbox.bind('<<ListboxSelect>>', on_component_select)
        device_listbox.bind('<<ListboxSelect>>', on_device_select)
        search_entry.bind('<Return>', lambda e: do_search())
        
        # Bottom buttons
        button_frame = ttk.Frame(editor)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        status_label = ttk.Label(button_frame, text=f"0 components mapped", foreground="blue")
        status_label.pack(side=tk.BOTTOM, pady=5)
        
        def update_status():
            count = len(current_mappings)
            total = len(comp_data)
            status_label.config(text=f"{count} of {total} components mapped")
        
        def apply_mappings():
            for ref, mapping in current_mappings.items():
                self.library_parser.add_manual_mapping(
                    ref, 
                    mapping['library'], 
                    mapping['deviceset'], 
                    mapping.get('device', '')
                )
            
            count = len(current_mappings)
            self._log(f"\n✓ Applied {count} component mappings")
            messagebox.showinfo("Success", f"Applied {count} component mappings!\n\nUse 'Save Mappings' to save to file for reuse.")
            editor.destroy()
        
        def clear_current():
            if current_component['ref']:
                ref = current_component['ref']
                if ref in current_mappings:
                    del current_mappings[ref]
                    selected_lib_var.set('Not mapped')
                    selected_dev_var.set('')
                    selected_device_var.set('')
                    # Reset highlight
                    idx = list(comp_data.keys()).index(ref)
                    comp_listbox.itemconfig(idx, bg='white')
                    update_status()
        
        btn_frame = ttk.Frame(button_frame)
        btn_frame.pack(side=tk.TOP)
        
        ttk.Button(btn_frame, text="Apply & Close", command=apply_mappings, width=20).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Clear Current", command=clear_current, width=20).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=editor.destroy, width=20).pack(side=tk.LEFT, padx=5)
        
        # Highlight already mapped components with correct colors
        for idx, ref in enumerate(comp_data.keys()):
            if ref in self.library_parser.manual_mappings:
                # Check if this was from auto-map or manually set
                # If in current_mappings already, it was loaded
                current_mappings[ref] = self.library_parser.manual_mappings[ref].copy()
                # Color as auto-mapped (light blue) initially
                comp_listbox.itemconfig(idx, bg='lightblue')
        
        update_status()
        
        # Track which components have been manually changed
        manually_changed = set()
    
    def _load_mappings(self):
        """Load component mappings from JSON file"""
        if not self.library_parser:
            messagebox.showerror("Error", "Please scan libraries first")
            return
        
        filename = filedialog.askopenfilename(
            title="Load Component Mappings",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            success, message = self.library_parser.load_manual_mappings(filename)
            if success:
                self._log(f"Loaded mappings: {message}")
                messagebox.showinfo("Success", message)
            else:
                self._log(f"Error loading mappings: {message}")
                messagebox.showerror("Error", message)
    
    def _save_mappings(self):
        """Save component mappings to JSON file"""
        if not self.library_parser:
            messagebox.showerror("Error", "No mappings to save")
            return
        
        filename = filedialog.asksaveasfilename(
            title="Save Component Mappings",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            success, message = self.library_parser.save_manual_mappings(filename)
            if success:
                self._log(f"Saved mappings: {message}")
                messagebox.showinfo("Success", message)
            else:
                self._log(f"Error saving mappings: {message}")
                messagebox.showerror("Error", message)
    
    def _validate_pin_count(self, ref: str, library: str, deviceset: str, device: str):
        """Validate that selected component has compatible pin count with connections"""
        if not self.schematic_data:
            return
        
        # Find all connections for this component in MD file
        md_pins = set()
        for conn in self.schematic_data.get('connections', []):
            if conn['from']['ref'] == ref:
                md_pins.add(conn['from']['pin'])
            if conn['to']['ref'] == ref:
                md_pins.add(conn['to']['pin'])
        
        if not md_pins:
            return  # No connections defined, can't validate
        
        max_md_pin = max(int(p) for p in md_pins if p.isdigit())
        
        # Check if component exists in library
        component_info: Optional[Dict] = None

        for entries in self.library_parser.device_map.values():
            if component_info:
                break
            if not isinstance(entries, list):
                entries = [entries]
            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                if (entry.get('library') == library and
                        entry.get('deviceset') == deviceset and
                        entry.get('device') == device):
                    component_info = entry
                    break
        
        if component_info and component_info.get('pins'):
            available_pins = component_info['pins']
            num_pins = len(available_pins)
            
            # Check if MD file uses pins that don't exist
            missing_pins = []
            for pin in md_pins:
                if pin not in available_pins:
                    missing_pins.append(pin)
            
            if missing_pins:
                warning = f"⚠️ Warning for {ref}:\n"
                warning += f"  MD file uses pins: {sorted(md_pins)}\n"
                warning += f"  Selected device has {num_pins} pins: {sorted(available_pins.keys())}\n"
                warning += f"  Missing pins: {missing_pins}\n"
                warning += f"  This may cause NET connection errors!"
                self._log(warning)
            elif max_md_pin > num_pins:
                warning = f"⚠️ Warning for {ref}:\n"
                warning += f"  MD file uses up to pin {max_md_pin}\n"
                warning += f"  Selected device only has {num_pins} pins\n"
                warning += f"  Check your connections!"
                self._log(warning)

    def _is_non_passive(self, ref: str) -> bool:
        if not ref:
            return True
        ref_upper = ref.upper()
        passive_prefixes = ('R', 'C', 'L', 'D', 'LED')
        return not any(ref_upper.startswith(prefix) for prefix in passive_prefixes)

    def _filter_pin_errors(self, errors: Optional[List[Dict]]) -> List[Dict]:
        filtered: List[Dict] = []
        seen = set()
        if not errors:
            return filtered
        for err in errors:
            ref = err.get('ref') if err else None
            if not ref or not self._is_non_passive(ref):
                continue
            pin_value = (err.get('pin') or '').strip()
            label_value = (err.get('label') or '').strip()
            if is_nc_identifier(pin_value) or is_nc_identifier(label_value):
                continue
            
            # Skip if this pin already has a valid override
            if self._has_valid_override(ref, err):
                continue
            
            key = (ref, pin_value, label_value)
            if key in seen:
                continue
            seen.add(key)
            filtered.append(err)
        return filtered
    
    def _has_valid_override(self, ref: str, issue: Dict) -> bool:
        """Check if pin error already has a valid override that resolved"""
        overrides = self.pin_overrides.get(ref, {})
        # Check if any of the pin identifiers (pin number or label) have an override
        for key in (issue.get('pin'), issue.get('label')):
            if key and key in overrides:
                # Override exists - consider it resolved
                return True
        return False

    def _get_existing_override(self, ref: str, issue: Dict) -> str:
        overrides = self.pin_overrides.get(ref, {})
        for key in (issue.get('pin'), issue.get('label')):
            if key and key in overrides:
                return overrides[key]
        return ''

    def _open_pin_mapping_dialog(self, pin_errors: List[Dict]):
        if not pin_errors:
            return
        if self._pin_mapping_window and self._pin_mapping_window.winfo_exists():
            self._pin_mapping_window.lift()
            return
        self._log("Opening pin mapping dialog to resolve skipped nets...")
        current_total = sum(len(pins) for pins in self.pin_overrides.values())
        self._log(f"Current in-memory overrides: {current_total}")
        window = tk.Toplevel(self.root)
        window.title("Resolve Pin Mapping Issues")
        window.geometry("820x620")
        window.grab_set()
        self._pin_mapping_window = window
        window.protocol("WM_DELETE_WINDOW", lambda: self._close_pin_mapping_window(window))
        ttk.Label(window, text=("Some non-passive components use Eagle pin names that differ from the Markdown file."
                                " Select the real library pin for each requested pin, then save."),
                  wraplength=690, justify=tk.LEFT).pack(padx=10, pady=10, anchor=tk.W)
        if self.pin_override_path:
            ttk.Label(window, text=f"Overrides file: {self.pin_override_path.name}", font=('Arial', 8, 'italic')).pack(padx=10, anchor=tk.W)
        notebook = ttk.Notebook(window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        issues_by_ref: Dict[str, List[Dict]] = {}
        for err in pin_errors:
            issues_by_ref.setdefault(err['ref'], []).append(err)
        entry_widgets: Dict[str, List[Dict]] = {}
        for ref, issues in issues_by_ref.items():
            frame = ttk.Frame(notebook)
            comp_value = issues[0].get('value') or 'Unknown value'
            tab_label = f"{ref} - {comp_value}"
            notebook.add(frame, text=tab_label)
            comp_header = f"{ref} — {comp_value}"
            ttk.Label(frame, text=comp_header, font=('Arial', 11, 'bold')).pack(anchor=tk.W, padx=8, pady=6)
            info_line = []
            lib = issues[0].get('library')
            deviceset = issues[0].get('deviceset')
            if lib and deviceset:
                info_line.append(f"Library: {lib}/{deviceset}")
            if issues[0].get('available'):
                info_line.append(f"Available pins: {', '.join(issues[0]['available'])}")
            if info_line:
                ttk.Label(frame, text=" | ".join(info_line), font=('Arial', 9), wraplength=660).pack(anchor=tk.W, padx=8)
            entry_widgets[ref] = []
            for issue in issues:
                row = ttk.Frame(frame)
                row.pack(fill=tk.X, padx=8, pady=4)
                pin_desc = issue.get('pin') or 'Unnumbered'
                label = issue.get('label')
                display = f"MD Pin {pin_desc}" + (f" ({label})" if label else '')
                ttk.Label(row, text=display, width=28).pack(side=tk.LEFT)
                options = list(issue.get('available') or [])
                lib_name = issue.get('library') or lib
                deviceset_name = issue.get('deviceset') or deviceset
                device_name = issue.get('device') or issues[0].get('device') or ''
                if self.library_parser:
                    if not lib_name or not deviceset_name:
                        placement = self.library_parser.find_component(ref, issues[0].get('value', ''), issues[0].get('package', ''))
                        if placement:
                            lib_name = lib_name or placement.get('library')
                            deviceset_name = deviceset_name or placement.get('deviceset')
                            device_name = device_name or placement.get('device', '')
                    if lib_name and deviceset_name:
                        self.library_parser.ensure_library_pins(lib_name)
                        lib_options = self.library_parser.get_device_pins(lib_name, deviceset_name, device_name)
                        if not lib_options:
                            self._log(f"⚠️ No pin metadata found for {ref} using {lib_name}/{deviceset_name} (variant '{device_name or 'default'}')")
                        else:
                            options.extend(lib_options)
                if options:
                    seen_options = []
                    for opt in options:
                        if opt not in seen_options:
                            seen_options.append(opt)
                    options = seen_options
                var = tk.StringVar(value=self._get_existing_override(ref, issue))
                state = 'readonly' if options else 'normal'
                combo = ttk.Combobox(row, values=options, textvariable=var, state=state, width=28)
                combo.pack(side=tk.LEFT, padx=5)
                entry_widgets[ref].append({'var': var, 'issue': issue, 'combo': combo})
        button_frame = ttk.Frame(window)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        def apply_mappings(mode: str):
            require_all = (mode == 'rerun')
            if require_all:
                missing = []
                for entries in entry_widgets.values():
                    for entry in entries:
                        combo_state = str(entry['combo'].cget('state'))
                        if combo_state == 'disabled':
                            continue
                        if not entry['var'].get().strip():
                            issue = entry['issue']
                            missing.append(f"{issue['ref']} {issue.get('pin') or issue.get('label')}")
                if missing:
                    messagebox.showerror("Missing Selection", "Select a library pin for each listed connection before re-running.")
                    return
            mappings_added = []
            for ref, entries in entry_widgets.items():
                for entry in entries:
                    issue = entry['issue']
                    combo_state = str(entry['combo'].cget('state'))
                    if combo_state == 'disabled':
                        continue
                    choice = entry['var'].get().strip()
                    if not choice:
                        continue
                    keys = [k for k in (issue.get('pin'), issue.get('label')) if k]
                    if not keys:
                        continue
                    override_map = self.pin_overrides.setdefault(ref, {})
                    for key in keys:
                        override_map[key] = choice
                        mappings_added.append((ref, key, choice))
            if mappings_added:
                self._log(f"Recorded {len(mappings_added)} pin override entry(ies)")
                for ref, key, choice in mappings_added:
                    self._log(f"  {ref}.{key} → {choice}")
            else:
                self._log("Pin override dialog closed without new selections")
            self._save_pin_overrides()
            current_total = sum(len(v) for v in self.pin_overrides.values())
            self._log(f"Total stored overrides: {current_total}")
            self._close_pin_mapping_window(window)
            if mode == 'rerun':
                self._log("Re-running SCR generation with manual pin overrides...")
                self.root.after(150, self._generate_scr)
        ttk.Button(button_frame, text="Save & Close", command=lambda: apply_mappings('save'), width=18).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Save & Re-run", command=lambda: apply_mappings('rerun'), width=18).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=lambda: self._close_pin_mapping_window(window), width=18).pack(side=tk.RIGHT, padx=5)

    def _close_pin_mapping_window(self, window):
        if window and window.winfo_exists():
            try:
                window.grab_release()
            except tk.TclError:
                pass
            window.destroy()
        self._pin_mapping_window = None

    def _edit_pin_overrides(self):
        if not self.pin_overrides:
            messagebox.showinfo("No Overrides", "No pin overrides have been recorded yet.")
            return

        editor = tk.Toplevel(self.root)
        editor.title("Edit Pin Overrides")
        editor.geometry("560x420")
        editor.grab_set()

        tree_frame = ttk.Frame(editor)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        columns = ("ref", "md_pin", "mapped_pin")
        tree = ttk.Treeview(tree_frame, columns=columns, show='headings', selectmode='browse')
        for col, title in zip(columns, ("Component", "MD Pin", "Mapped Eagle Pin")):
            tree.heading(col, text=title)
            tree.column(col, width=170 if col != "mapped_pin" else 190, anchor=tk.W)

        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(tree_frame, orient='vertical', command=tree.yview)
        tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        def refresh_tree():
            tree.delete(*tree.get_children())
            for ref in sorted(self.pin_overrides.keys()):
                overrides = self.pin_overrides.get(ref, {})
                for md_pin, mapped in sorted(overrides.items()):
                    tree.insert('', tk.END, values=(ref, md_pin, mapped))

        refresh_tree()

        edit_frame = ttk.Frame(editor)
        edit_frame.pack(fill=tk.X, padx=10, pady=5)

        selected_ref = tk.StringVar()
        selected_md = tk.StringVar()
        new_pin_var = tk.StringVar()

        ttk.Label(edit_frame, text="Component:").grid(row=0, column=0, sticky=tk.W, padx=2, pady=2)
        ttk.Label(edit_frame, textvariable=selected_ref).grid(row=0, column=1, sticky=tk.W, padx=2, pady=2)

        ttk.Label(edit_frame, text="MD Pin/Label:").grid(row=1, column=0, sticky=tk.W, padx=2, pady=2)
        ttk.Label(edit_frame, textvariable=selected_md).grid(row=1, column=1, sticky=tk.W, padx=2, pady=2)

        ttk.Label(edit_frame, text="Mapped To (Eagle pin):").grid(row=2, column=0, sticky=tk.W, padx=2, pady=2)
        entry = ttk.Entry(edit_frame, textvariable=new_pin_var, width=28)
        entry.grid(row=2, column=1, sticky=tk.W, padx=2, pady=2)

        status_var = tk.StringVar(value="Select an override to edit or delete.")
        status_label = ttk.Label(editor, textvariable=status_var, foreground="blue")
        status_label.pack(fill=tk.X, padx=10)

        def on_select(event=None):
            selection = tree.selection()
            if not selection:
                selected_ref.set("")
                selected_md.set("")
                new_pin_var.set("")
                status_var.set("Select an override to edit or delete.")
                return
            values = tree.item(selection[0], 'values')
            selected_ref.set(values[0])
            selected_md.set(values[1])
            new_pin_var.set(values[2])
            status_var.set("Edit the Eagle pin value and click Save Change.")
            entry.focus_set()

        tree.bind('<<TreeviewSelect>>', on_select)

        def save_change():
            ref = selected_ref.get()
            md_pin = selected_md.get()
            new_pin = new_pin_var.get().strip()
            if not (ref and md_pin):
                messagebox.showerror("No Selection", "Select an override first.")
                return
            if not new_pin:
                messagebox.showerror("Invalid Pin", "Enter the Eagle pin name to map to.")
                return
            overrides = self.pin_overrides.setdefault(ref, {})
            overrides[md_pin] = new_pin
            self._save_pin_overrides()
            self._log(f"Updated pin override: {ref}.{md_pin} -> {new_pin}")
            refresh_tree()
            status_var.set("Override updated and saved.")

        def delete_selected():
            ref = selected_ref.get()
            md_pin = selected_md.get()
            if not (ref and md_pin):
                messagebox.showerror("No Selection", "Select an override to delete.")
                return
            if messagebox.askyesno("Delete Override", f"Remove override for {ref} {md_pin}?"):
                overrides = self.pin_overrides.get(ref, {})
                if md_pin in overrides:
                    overrides.pop(md_pin)
                    if not overrides:
                        self.pin_overrides.pop(ref, None)
                    self._save_pin_overrides()
                    self._log(f"Deleted pin override: {ref}.{md_pin}")
                    refresh_tree()
                    selected_ref.set("")
                    selected_md.set("")
                    new_pin_var.set("")
                    status_var.set("Override deleted.")

        def close_editor():
            try:
                editor.grab_release()
            except tk.TclError:
                pass
            editor.destroy()

        button_frame = ttk.Frame(editor)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        ttk.Button(button_frame, text="Save Change", command=save_change, width=16).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Delete Override", command=delete_selected, width=16).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close", command=close_editor, width=16).pack(side=tk.RIGHT, padx=5)

        editor.protocol("WM_DELETE_WINDOW", close_editor)


def main():
    """Main entry point"""
    root = tk.Tk()
    app = EagleSchematicGeneratorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
