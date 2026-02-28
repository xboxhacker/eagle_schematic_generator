# AI MD Schematic Prompt

Use this file when you need another AI model to draft a Markdown schematic that `eagle_schematic_generator.py` can parse without errors. Copy the block below, paste it into your AI chat, replace `<PROJECT_SPEC>` with your design notes, and ask the AI to produce the final `.md` document exactly once.


```text
You are creating a Markdown schematic specification that will be parsed by the script `eagle_schematic_generator.py`. Use the design details I provide in <PROJECT_SPEC>, but ALWAYS obey the formatting and pin rules below so the script can build the Eagle schematic file without manual fixes.

Assume nothing—read or recall the real component datasheets for EVERY IC and active device. You need the datasheet for two critical things: (1) the correct physical pin numbers and their functional names, and (2) the exact package type (DIP, SOIC, QFN, TQFP, etc.) for the specific part number. Different suffixes on the same IC family indicate different packages — for example, 74HC14**N** = DIP-14 but 74HC14**D** = SOIC-14, and LM7805**CT** = TO-220 but LM7805**MP** = SOT-223. Always check what the part number suffix means and put the correct package in the BOM.

This instruction set must work with any AI model and any electronics design. Explicitly restate every device's formal name or manufacturer part number, its electrical value/specification, and enumerate **all** of its connections so nothing is implicit.

1. Document outline
   - Begin with `# <Project Name>` followed by a short subtitle.
   - Add a quick bullet summary (configuration, part count, power notes).
   - Organize the rest of the document with `## SECTION <n>: <Title>` blocks so each functional group is obvious.

2. Bill of Materials (BOM)
   - Create a heading `## COMPLETE BILL OF MATERIALS` (exact text).
   - Provide ONE Markdown table whose first column is `Ref`. Other columns must include `Qty`, `Part Number`, `Value`, `Description`, and `Package`.
   - Keep reference designators uppercase (U1, R3, J5, etc.). Use comma-separated refs (e.g., `R1, R2`) only when they truly share the same part.
   - **CRITICAL for passive components (R*, C*, L*)**: The `Value` column MUST contain the electrical value (e.g., `10kΩ`, `100nF`, `4.7µH`). This value will appear on the schematic symbol in Eagle. Use standard notation: `kΩ`, `MΩ` for resistors; `pF`, `nF`, `µF` for capacitors; `nH`, `µH`, `mH` for inductors.
   - **Capacitor type**: Always specify whether a capacitor is **electrolytic** or **ceramic** in the `Description` column. This affects the Eagle library symbol selection (polarized vs. non-polarized). Use `Electrolytic` for larger values (typically ≥1µF) where polarity matters, and `Ceramic` for smaller decoupling/bypass caps (typically ≤1µF). Example: `| C1 | 1 | Generic | 10µF | 10µF Electrolytic | RAD-0.2 |` vs. `| C2 | 1 | Generic | 100nF | 100nF Ceramic | 0805 |`. If the design requires a specific type regardless of value, state it explicitly.
   - **CRITICAL — Package column for ICs and active components**: You MUST consult the manufacturer's datasheet to determine the exact package type for every IC. Do NOT guess — different packages have different pin counts, pin assignments, and thermal pads. Use the standard package name from the datasheet in the `Package` column:
     - **Through-hole**: `DIP-14`, `DIP-16`, `DIP-18`, `DIP-28`, `TO-220`, `TO-92`, etc.
     - **SMD gull-wing (leads on sides)**: `SOIC-14`, `SOP-16`, `SSOP-20`, `TSSOP-16`, `MSOP-8`, `QFP-44`, `LQFP-48`, `TQFP-64`, etc.
     - **SMD no-lead / pad-underneath**: `QFN-16`, `DFN-8`, `LFCSP-32`, `WSON-8`, etc.
     - **SMD area-array**: `BGA-256`, `LGA-48`, `CSP-36`, etc.
     - **Other SMD**: `PLCC-28`, `SOT-23`, `SOT-223`, `SC-70`, etc.
     - The format is always `TYPE-N` where N is the total pin count (including exposed/thermal pads if present). The generator uses this to pick the correct package variant from the library.
     - If the project spec says "use the DIP version" or "through-hole only," use `DIP-N`. If no preference is stated, check what package the specified part number actually comes in and use that.
   - For active components (ICs, connectors, etc.), the `Value` column can contain the part number or functional description.
   - Example BOM table format:
     ```
     | Ref | Qty | Part Number | Value | Description | Package |
     |-----|-----|-------------|-------|-------------|---------|
     | U1 | 1 | AD2428W | AD2428W | A2B Transceiver | LFCSP-32 |
     | U2 | 1 | 74HC14N | 74HC14 | Hex Schmitt Inverter | DIP-14 |
     | U3 | 1 | ATMEGA328P-AU | ATMEGA328P | Microcontroller | TQFP-32 |
     | Q1 | 1 | IRLZ44N | IRLZ44N | N-Channel MOSFET | TO-220 |
     | R1, R2 | 2 | Generic | 10kΩ | 10kΩ | AXIAL |
     | C1 | 1 | Generic | 100nF | 100nF Ceramic | RADIAL |
     | C2 | 1 | Generic | 10µF | 10µF Electrolytic | ELECTROLYTIC |
     | J1 | 1 | 104087-0801 | 8-Pin Header | Molex Connector | - |
     ```
   - Every component that appears anywhere else in the document MUST exist in this table.

3. Component sections
   - For every reference in the BOM, create a heading in the form `### <REF> - <Description>`.
   - **For passive components (R*, C*, L*)**: Use the electrical VALUE as the description. Examples:
     - `### R1 - 10kΩ` (not "Pull-up resistor")
     - `### C1 - 100nF` (not "Decoupling capacitor")
     - `### L1 - 4.7µH` (not "Filter inductor")
     This ensures the value appears on the Eagle schematic symbol.
   - **For active components**: Use part number or descriptive name. Examples:
     - `### U1 - AD2428 A2B Transceiver`
     - `### J1 - 8-Pin Molex Connector`
   - Immediately follow the heading with a fenced code block (``` on its own line). Inside the block list the pin-by-pin details for that component.
   - For every non-passive component (anything whose reference does **not** start with R, C, L, D, or LED—including all U*, J*, connectors, crystals, regulators, ICs, etc.), each pin line MUST use the format `Pin <number> (<datasheet-name>): ...` even if the pin is NC. Example: `Pin 4 (BCLK): Reference clock ─── U2.Pin13 (BCK)` or `Pin 7 (NC): Not connected`.
   - **CRITICAL for ICs (U*)**: Use the **physical pin number** from the datasheet (1, 2, 3... 14, 16, etc.). The parenthetical name must match the **Eagle library logical pin name** (e.g. 1A, 1Y, 2A, VCC, GND) so the generator can map correctly. Example: for 74HC08 use `Pin 3 (1Y)` not `Pin 3 (OUT1)`—Eagle uses 1Y. For active-low pins, Eagle uses `~` (e.g. `1~A`, `2~Q`); in the MD file you may write the name without the tilde (e.g. `1A`, `2Q`)—the generator strips tildes during matching. Check the target library's pin names when in doubt.
   - Passive parts may continue to use just pin numbers (1/2) because most libraries expose simple numbered pads.
   - Each pin line must start with the `Pin <number> (<name>)` prefix followed by a short label, then the connection path using the Unicode line `─` (U+2500). Example: `Pin 2 (SIG_P): Differential positive ─── J2.Pin2 (SIG_P)`.
   - When the design assigns a named net to that pin, inject the literal token `Net <NET_NAME>` immediately before the path (for example: `Pin 2 (SIG_P): Net DIFF_POS (positive data) ─── J2.Pin3`). This keeps the prose readable and guarantees the parser sees the official net label.
   - When referencing another component, ALWAYS use the exact token `<OtherRef>.Pin<digits>` (capital ref, literal period, literal `Pin`, numeric digits). This exact spelling is what the parser looks for.

4. Connection syntax rules the parser depends on
   - Use only numeric pin identifiers (no `A`, `TP`, or `PAD` labels) unless the actual Eagle device uses that exact alphanumeric pin name.
   - Maintain the `Pin <num> (<name>):` prefix even inside ASCII diagrams for non-passives, and include the `<name>` pulled directly from the datasheet so another engineer can confirm it instantly.
   - Use three or more `─` characters (U+2500) between nodes so the regex `[──]+` matches. The preferred connection format inside a component block is:
     `Pin 1 (LED1): J7.Pin2 (LED1)` — direct reference (simplest)
     `Pin 3 (1Y) ─── J3.Pin3 (C_TRIGGER)` — separator style (also works)
     Both formats are fully supported. The key requirement is that the target `CompRef.PinN` token appears on the same line as the source pin.
   - **Never** describe a connection using only prose like "connects to the header" — always use `CompRef.PinN` notation on the pin line.
   - Every time you mention a named signal, reuse the same `Net <NAME>` spelling everywhere (component pins, ASCII diagrams, and summary tables) so the parser can unify those nodes automatically.
   - When a connection is described outside of a component-specific block, write it as `<RefA>.PinX ─── <RefB>.PinY` on a single line.
   - Describe every electrical connection explicitly at least once—no implied links, no "same as above" wording—so the parser sees 100% of the nets.
   - Do NOT mirror the same net in both component sections; pick the most logical source and reference the destination from there.
   - **CRITICAL — exhaustive pin listing**: If a component (e.g., an LM3914 display driver or an LED bar connector) has many output pins that each wire to a different target, list EVERY pin on its own line with the explicit `CompRef.PinN` target. Do NOT summarize groups of pins or skip "obvious" connections. Example for an 18-pin IC with 10 LED outputs:
     ```
     Pin 1 (LED1): J7.Pin2
     Pin 10 (LED10): J7.Pin11
     Pin 11 (LED9): J7.Pin10
     ... (one line per pin, no exceptions)
     ```

5. Pin-accuracy checklist (never invent pins)
   - For every component, copy the exact **physical pin numbers** (1…N) from the manufacturer datasheet. For ICs, the number in `Pin 14 (VCC)` is the physical package pin—Eagle maps logical names (VCC, 1A, GND) to these numbers internally. Use the datasheet pinout diagram; never guess.
   - If the symbol exposes only three pads, you may only reference those three pads.
   - Whenever a pin also has a functional name (BCLK, SDA, VOUT, SHIELD, etc.), include that name in parentheses after the pin number exactly as it appears in the datasheet/symbol.
   - Connectors must stay within their physical pin count (e.g., a 1x4 header exposes pins 1-4, a barrel jack exposes three pins). Do **not** invent higher numbers or duplicate pins that the library part does not provide.
   - ICs (U*), regulators, sensors, oscillators, and any other multi-pin active device must list every used pin with both number and name; unused pins should be explicitly marked as `NC` if the datasheet labels them that way.
   - If you repurpose a connector pin for a different function (for example, using a generic USB pin as a differential bus pair), update the parenthetical pin name to the **actual** signal you are routing (e.g., `Pin 3 (SIG_P)` instead of legacy `D+`). Never leave outdated names that don’t exist in the target IC.
   - Passive parts (R*, C*, L*, D*, LED*) must reflect the true pad count for the package you specify. Example: standard resistors are pin 1–2, common‑mode chokes may expose pins 1–4, radial capacitors may expose pins 1–2 with polarity noted.
   - Diode-class devices (D* references, TVS arrays, CAN suppression networks) may expose Eagle pins literally named `+` or `-` in addition to numeric pads; pull the datasheet, list both the sign-based label and the physical pin number, and explicitly state which side is the anode or cathode so the script can map it.
   - LED references (LED*) often expose pads labeled `A` and `C` alongside numeric pins; include the pad letter, the pin number, and explicitly call out which node is the anode vs. cathode when you describe every connection.
   - Before finishing, scan the document and confirm every `ComponentRef.Pin#` token references a REAL pin for that device and appears somewhere in the BOM table.
   - Double-check that the `<name>` you publish next to each pin number matches the datasheet and the Eagle symbol you expect to use. If you are unsure, stop and look it up—never guess.
   - Whenever a pin ties into a power rail or shield, explicitly write `Net GND`, `Net VIN`, `Net 3V3`, etc. (use the real rail name) so the generator can place that connection on the shared bus layer. Generic text like “connect to ground” is not enough—always emit the `Net <NAME>` token.

6. Nets and power rails
   - Document **every** named net in its own summary line using the exact string `Net <NAME>: <Ref.Pin list>` toward the end of the file. Include every participant once so the parser has an authoritative roster.
   - **For ICs and connectors**: In the Net summary, optionally add the datasheet pin name in parentheses after the pin number to improve automatic pin mapping. Example: `Net COIL1_TRIG: U4.Pin3 (1Y), J3.Pin3 (C_TRIGGER)`. This helps the generator match MD pins to Eagle library logical names (1A, 1Y, VCC, etc.).
   - Within the body, feel free to add human-friendly notes, but always prefix the connection clause with `Net <NAME>` so readers can map prose→net.
   - Keep net names uppercase alphanumeric (VDD, +12V, GND, A2BP, etc.).
   - Use the same `Net <NAME>` spelling everywhere (pin tables, ASCII diagrams, and summary lists). If you rename the rail midway, the generator creates separate nets and you will lose the shared blue bus in Eagle.

7. Quality checks before responding
   - Ensure every BOM reference appears at least once in the component sections and every component section refers back to the BOM.
   - Verify there are no duplicate or conflicting reference designators.
   - Verify every connector, jack, header, and IC stays within its legitimate pin range (no phantom pins or mislabeled pads).
   - Confirm there are enough connection statements (the script needs dozens of `Pin` lines to build nets) and that every device lists all of its pins that the design actually uses.
   - **Connection completeness**: For every pin that connects to another component, the MD file MUST contain an explicit `CompRef.PinN` reference somewhere — either as a separator-style connection (`Pin 3 (1Y) ─── J3.Pin3`) or as a direct reference (`Pin 3 (1Y): J3.Pin3`). If a component has 18 pins and 15 connect to other components, verify all 15 have explicit `CompRef.PinN` tokens. The parser counts every connection — if you miss one, that wire will be absent from the schematic.
   - **Do not rely on Net summaries alone for connections.** The Net summary section at the bottom is a cross-reference for humans and the net-merger; the parser's primary connection data comes from the pin lines inside each component's `### RefN` section. Every connection must appear in the component section.
   - **CRITICAL — Cross-reference consistency**: When a connection appears in two different component sections, the pin numbers MUST agree. For example, if U6's section says `Pin 10 (LED10): J7.Pin2` then J7's section MUST say `Pin 2 (LED10): U6.Pin10` — NOT `Pin 3: U6.Pin10`. An off-by-one error here silently cascades: it puts one IC pin on two different connector pins, which the parser merges into one net, which then daisy-chains every subsequent connection onto that same net. This is especially dangerous with multi-pin connectors that have a power/common pin at Pin 1 — if you accidentally map IC outputs starting at Pin 1 instead of Pin 2, every output gets shorted to the supply rail. **After writing all connections, cross-check every CompRef.PinN target against the target component's own section to confirm the pin numbers match.**

8. Output expectations
   - Return a single cohesive Markdown document, not separate answers per section.
   - Do not include pseudo-code or TODOs; everything must be final.
   - Use only ASCII text except for the `─` character that the parser requires.

<PROJECT_SPEC>
```
