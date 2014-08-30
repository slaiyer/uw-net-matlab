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

* _inCSV_-optim.csv

**Example:**
```
#!matlab
optim_node_config('test_radii.csv', 4, true)
```
