# Very Important !!
## Create a virtual environment before running any of this code
## Source the workspace parent directory
```sh
source install/setup.bash
```

## Performance Metrics

| Script    | Average Simulation Time (minutes) |
| :-------- | :-------: |
| qr_test.py (with axis coupling)  | 3.092 | 
| qr_test.py (no axis coupling) | 1.784 |


## Individual Metrics

### With Axis Coupling
| Script    | Test 1 | Test 2 | Test 3 | Test 4 | Test 5 |
| :-------- | :-------: | :-------: | :-------: | :-------: | :-------: |
| qr_test.py | 2.29 | 2.43 | 2.49 | 4.51 | 3.74 |


### No Axis Coupling 
| Script    | Test 1 | Test 2 | Test 3 | Test 4 | Test 5 |
| :-------- | :-------: | :-------: | :-------: | :-------: | :-------: |
| qr_test.py | 1.92 | 1.82 | 1.83 | 1.68 | 1.67 |