**Entry point:**

* optim\_config\_node(_inCSV_, _iters_, _verbose_)

**Input:**

1. _inCSV_: Path to CSV file with node radii

2. _iters_ (Optional)

    * Default: 1

    * Number of times stretch\_chainlink is called with the same _R_

3. _verbose_ (Optional)

    * Default: false

    * Controls the verbosity of optim\_config\_node and stretch\_chainlink

**Output:**

1. Best node configuration: _inCSV_-optim_node_config.csv

2. Maximal volume achieved: _inCSV_-optim_vol.txt

**Example:**

```
#!matlab
optim_node_config('test_radii.csv', 4, true)
```

**Documentation:**

* MATLAB-generated documentation from the source code is in the `html` directory.