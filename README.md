# Caravel User Project

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

| :exclamation: Important Note            |
|-----------------------------------------|

# OSU gf180mcuC 3.3V standard cell caravel test - ffra

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml)


## An 32-bit ffra adder implementation using OSU standard cells

This design is an adder implementation of Anton Blanchard's [vlsiffra](docs/source/index.rst) architecture. The purpose of this is to make OpenLane's gf180mcuC pdk and overall flow compatible with a 3.3V standard cell kit, as well as to test the functionality of OSU's standard cell library for the gf180 process node.

The use of this design is very simple, and should be compatible with any external controller. Caravel's GPIO is setup exclusively with user input settings, so that the IO of the chip and be interacted with directly.
