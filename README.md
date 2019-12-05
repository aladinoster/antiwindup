**| [Overview](#overview) | [Reproducibility](#reproducibility) | [License](#license) | [Contact](#contact) |**

# Antiwindup PID system

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/aladinoster/antiwindup.git/master?filepath=antiwindup.ipynb)

## Overview

This implements an antiwindup system.

![](img/antiwindup.png)

### Base functions 

This implementation contains a regular `PID` a `PID` with bounded output and a `PID` with anti-windup system.

#### Usage 

The code for the PID is in `pid.py`

```{python}
from pid import PID, PIDantiwindup

TS = 0.1 # Sampling time 
pid_normal = PID(k_p=1,k_i=0.5,k_d=0.2) # Creates a regular PID 
pid_windup = PIDantiwindup(k_p=1,k_i=0.5,k_d=0.2) # Creates a regular PID 

# Usage (operates over single values), initial conditions are 0 
error = 0.2
u = pid_normal(error)
u = pid_windup(error) 
```

## Reproducibility

Download this repository

```{bash}
git clone https://github.com/aladinoster/antiwindup.git
```

Be sure to get [conda](https://www.anaconda.com/distribution/), then:

```{bash}
conda env create -f environment.yml
conda activate antiwindup
jupyter lab antiwindup.ipynb
```

## License

The code here contained is licensed under [MIT License](LICENSE)

## Contact 

If you run into problems or bugs, please let us know by [creating an issue](https://github.com/aladinoster/antiwindup/issues/new) an issue in this repository.