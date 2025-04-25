# Multiverse Parser

The **Multiverse Parser** module enables conversion between different scene description formats, using **USD** (Universal Scene Description) as a common translation medium.

## Prerequisites

- Python ≥ 3.10
- Python packages listed in [requirements.txt](https://github.com/Multiverse-Framework/Multiverse-Parser/blob/main/requirements.txt), which can be installed with:

```bash
pip install -r requirements.txt
```

---

## Setup

First, download or clone the repository:

```bash
git clone https://github.com/Multiverse-Framework/Multiverse-Parser.git --depth 1
```

Then, run the [setup.sh](https://github.com/Multiverse-Framework/Multiverse-Parser/blob/main/setup.sh) script to download and link [Blender](https://www.blender.org/) automatically:

```bash
./Multiverse-Parser/setup.sh
```

To additionally build USD (optional), run:

```bash
./Multiverse-Parser/setup.sh --usd
```

---

## Usage

```bash
./Multiverse-Parser/multiverse_parser --help
```

```bash
usage: multiverse_parser [-h] --input INPUT --output OUTPUT [--physics | --no-physics] [--visual | --no-visual] [--collision | --no-collision]
                         [--keepusd | --no-keepusd] [--collisionrgba COLLISIONRGBA [COLLISIONRGBA ...]]

Multiverse parser

options:
  -h, --help            show this help message and exit
  --input INPUT         Import scene description as (URDF, MJCF, WORLD or USD)
  --output OUTPUT       Export scene description as (URDF, MJCF, WORLD or USD)
  --physics, --no-physics
                        Whether to include physics properties or not
  --visual, --no-visual
                        Whether to include visual meshes or not
  --collision, --no-collision
                        Whether to include collision meshes or not
  --keepusd, --no-keepusd
                        Whether to keep the USD file or not
  --collisionrgba COLLISIONRGBA [COLLISIONRGBA ...]
                        The color of the collision meshes, if they exist
```
