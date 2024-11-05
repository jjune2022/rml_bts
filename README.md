# logistics-single-vehicle
Logistics optimization algorithm for single vehicle.

### Solver requirements
* Gurobipy version: 10.0.3
#### Install gurobipy
```python
python -m pip install gurobipy==10.0.3
```

### My own python modules
It is for using my route planning algorithm (```planner.cpp```) in Python3. 
pybind11 is a header library facilitating seamless C++ and Python interoperability with minimal code, 
making it straightforward to expose C++ functions and classes to Python.
#### Install pybind11
Using APT,
```python
sudo apt update
sudo apt install pybind11-dev
```
#### Compile
```python
c++ -O3 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) astar.h astar.cpp planner.cpp -o planner.so
```

### Run demo code
```pyton
python3 main.py -call ${delivery_location} {number_of_delivered_items} -t ${working_time} -c ${capacity} -penalty ${penalty}
```

### Example
* Delivery calls

```python
python3 main.py -call 18 6 -call 45 8 -call 18 4 -call 1 5 -penalty 10
```
