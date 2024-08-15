<!-- <div align="center">
<img src="doc/image/IR_SIM_logos/logo1_nobg.png" width = "200" >
</div>  -->


<div align="center">

# Intelligent Robot Simulator (IR-SIM)

<a href="https://img.shields.io/badge/release-v2.1.0-brightgreen?link=https%3A%2F%2Fgithub.com%2Fhanruihua%2Fir_sim%2Freleases%2F
)](https://github.com/hanruihua/ir_sim/releases/"><img src='https://img.shields.io/github/v/release/hanruihua/ir_sim?color=brightgreen' alt='Github Release'></a>
<a href="https://github.com/hanruihua/ir_sim?tab=MIT-1-ov-file"><img src='https://img.shields.io/badge/License-MIT-blue' alt='License'></a>
<a href="https://pypistats.org/packages/ir-sim"><img src='https://img.shields.io/pypi/dm/ir_sim' alt='Download'></a>

</div>

IR-SIM is an open-source, lightweight robot 2D simulator based on Python, specifically designed for intelligent robotics navigation and learning. Primarily intended for research and educational purposes, it is user-friendly and easily customizable.

It provides the following features:
  - A versatile and easy-to-use framework for simulating a variety of robot platforms with kinematics and sensors. 
  - Customizable configurations and parameters using yaml files.
  - Real-time visualization of simulation outcomes.
  - Ideal for developing and testing algorithms related to robot navigation, motion planning, reinforcement learning.


Robot             |  Car
:-------------------------:|:-------------------------:
![robot](doc/animations/rvo.gif)  |  ![car](doc/animations/car.gif)


## Prerequisite

- Python: >= 3.7

## Installation

- Install this package from PyPi:

```
pip install ir_sim
```

- Or for development, you may install from source: 

```
git clone https://github.com/hanruihua/ir_sim.git    
cd ir_sim   
pip install -e .  
```

## YAML Configuration Example

```yaml

world:
  height: 10  
  width: 10   
  step_time: 0.1  
  sample_time: 0.1   
  offset: [0, 0] 
  collision_mode: 'stop'  # 'stop', 'unobstructed', 'reactive'
  control_mode: 'auto'  # 'auto', 'keyboard'

robot:
  - number: 10
    distribution: {name: 'circle', radius: 4.0, center: [5, 5]}  # name: 'circle', 'random',
    kinematics: {name: 'diff'} # name: 'diff', 'omni', acker
    shape: 
      - {name: 'circle', radius: 0.2}  # name: 'circle', 'rectangle'
    behavior: {name: 'rvo', vxmax: 1.3, vymax: 1.3, accer: 1.0, factor: 0.5} # name: 
    vel_min: [-2, -2.0]
    vel_max: [2, 2.0]
    color: ['royalblue', 'red', 'green', 'orange', 'purple', 'yellow', 'cyan', 'magenta', 'lime', 'pink', 'brown'] 
    arrive_mode: position   # position, state
    goal_threshold: 0.15
    plot:
      show_trail: true
      show_goal: true
      trail_fill: True
      trail_alpha: 0.2

obstacle:
  - shape: {name: 'circle', radius: 1.0}  
    state: [5, 5, 0]  
  
  - number: 10
    distribution: {name: 'manual'}
    shape:
      - {name: 'polygon', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2], irregularity_range: [0, 1], spikeyness_range: [0, 1], num_vertices_range: [4, 5]}  

  - shape: {name: 'linestring', vertices: [[5, 5], [4, 0], [1, 6]] } 
    state: [0, 0, 0] 

```
    
## Usage

### Quick Start

```python

from ir_sim.env import EnvBase

env = EnvBase('robot_world.yaml')

for i in range(300):

    env.step()
    env.render()

    if env.done(): break
        
env.end()
```

### Advanced Usage

The advanced usages are listed in the [ir_sim/usage](https://github.com/hanruihua/ir_sim/tree/main/ir_sim/usage)


## Cases
- [rl-rvo-nav](https://github.com/hanruihua/rl_rvo_nav)(RAL & ICRA2023)
- [RDA_planner](https://github.com/hanruihua/RDA_planner)（RAL & IROS2023）


<!-- ## Contact: 
hanrh@connect.hku.hk -->

<!-- ## Citation

```
@misc{ir_sim,
 author = "Ruihua Han",
 title = "ir-sim: Python based light-weight simulator for robotics navigation and learning.",
 year = 2024,
 url = "https://github.com/hanruihua/ir_sim"
}
``` -->





