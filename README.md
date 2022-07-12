# Evaluator

This module implements following critreions,

| Criteria | Description |
| -------- | ----------- |
| Time to explore | Measures the time taken to complete an exploration task. This reflects the suitability for time critical and limited power scenarios |
| Exploration path length | Measures the distance travelled during an exploration task. This reflects the suitability for power limited scenarios |
| Lateral Stress | Indicates the stress on the robot in the lateral direction with respect to the direction of travel and reflects the stability of the robot |This is calculated by integrating the instantaneous centripetal force along the trajectory |
| Tangential Stress | Indicates the stress on the robot along the direction of travel and reflects the stability of the robot. This can be calculated by integrating the absolute instantaneous linear acceleration along the trajectory |
| Explored Percentage | Indicates the completeness of the autonomous exploration task compared to a manual exploration task using the same robot and same sensors |

IF you use this package, Please cite

```sh
@INPROCEEDINGS{Ratnayake2021,
  author={Ratnayake, Kalana and Sooriyaarachchi, Sulochana and Gamage, Chandana},
  booktitle={2021 5th International Conference on Robotics and Automation Sciences (ICRAS)}, 
  title={OENS: An Octomap Based Exploration and Navigation System}, 
  year={2021},
  volume={},
  number={},
  pages={230-234},
  doi={10.1109/ICRAS52289.2021.9476592}}
```
