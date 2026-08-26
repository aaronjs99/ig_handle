# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| hardware.yaml | Defines the disabled-until-measured single-NC-limit telescope contract, shared BTS enable, quadrature encoder, active-high D27 field-valid supervisor, falling-edge enable removal with 10 ms polling backup, required independently tested hard E-stop in the 12 V motor path, mechanics, and safety gates. | measured telescope geometry, field rail, hard E-stop test, and wiring | Manual synchronization with ../teensy/firmware_config.h, ../../docs/telescope.md, commissioning work |
