# Experiments for Tracking Fake Base Stations

This repository contains the experiments used to generate test data for the setup introduced in the paper "Network-based Tracking of Fake Base Stations with Directional Antenna", submitted to Communications in Computer and Information Science (CCIS), by Tuomo Lehtilä, Sanish Gurung, Mohamed Taoufiq Damir, Amy Sokhna Sidibé, Gizem Akman, and Valtteri Niemi.

## ns-3 simulator

We used the ns-3 simulator to generate the test data. ns-3 can be downloaded from its [webpage](https://www.nsnam.org/docs/release/3.48/installation/html/index.html) and installed by following the official installation guide. For a quicker setup, see the [Quick Start guide](https://www.nsnam.org/docs/release/3.48/installation/html/quick-start.html).

To install the 5G-LENA `nr` module, follow the steps in the [nr module installation guide](https://cttc-lena.gitlab.io/nr/html/index.html).

## Generating test data

Once ns-3 and the `nr` module are installed and built, locate the `scratch` directory inside your ns-3.48 folder. Copy or move the `post-publication.cc` file into this directory.

This C++ file generates the synthetic test data. You can control the randomness of the generated datasets by adjusting the `seed` and `run` variables inside the `main` function. Changing these values alters the initial positions of the UEs, BSs, and FBS, allowing you to produce multiple distinct test datasets. Note that the legitimate BS positions are predefined for each seed, so `seed` must be one of 11, 22, 33, 44, or 55.

The propagation model is selected with the `propagation` variable (`true` for 3GPP UMa, `false` for COST 231), and the simulation duration with `simTime`.

## Running the ns-3 code

Once `post-publication.cc` is placed in the `scratch` folder, follow these steps to run it:

1. Open the terminal.
2. Navigate to your ns-3 directory (e.g., `ns-allinone-3.48/ns-3.48`) using the `cd` command.
3. Run the C++ file by executing: `./ns3 run scratch/post-publication`

After the simulation finishes, a new output file will appear in the ns-3 directory. Its name reflects the simulation settings, for example:

```
localization propagation=Cost231PropagationLossModel time=5 grid=1000X1000 n(UE)=300 n(gNB+FBS)=8 lbs=48 fbs=50 seed=55 run=1.txt
```

## GitHubMathematicaHP.txt 
This file contains Mathematica code for computing half-plane scores, trilaterating UE locations and postprocessing the FBS to directional antenna. When running this file, desired directional angle for the BS should be chosen. It has been created in Mathematica version 12. 

## ScoringEfficient.ipynb 
This file contains python code for computing the convex hull scores. 

## GitHubTimeIntervalMathematica.txt 
This file contains Mathematica code for computing time window based FBS localizatio. It has been created in Mathematica version 12.

The order of running these files is first GitHubMathematicaHP.txt, then ScoringEfficient.ipynb and finally GitHubTimeIntervalMathematica.txt
