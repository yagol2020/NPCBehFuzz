# NPCBehFuzz
Safety-Critical Driving Scenario Generation based on NPC Behavior Management

![overall](./assets/overall.png)


## Setup

1. Pull the CARLA simulator

```shell
docker pull carlasim/carla:0.9.13
```

2. Create a Python enviroments by Anaconda.
```shell
conda create -n npcbehfuzz python=3.7
```

3. Install Python depency.
```shell
pip install carla==0.9.13
pip install loguru matplotlib seaborn numpy
```

4. Install depency requirement by `InterFuser`
```shell
cd InterFuser
pip install -r requirements.txt
cd interfuser_code
pip install -r requirements.txt
python setup.py develop
```

5. Download the model weight of `InterFuser`, please refer to the [repositoyr](https://github.com/opendilab/InterFuser) of `InterFuser`
```shell
cp interfuser.pth.tar InterFuser/team_code/interfuser.pth.tar
```

## Fuzzing with `NPCBehFuzz`

```shell
conda activate npcbehfuzz
python main.py npcbehfuzz 2 # 2 NPC during the simulation
```

### Check the collision report
The collision reports and video recoding is saved in dirction `results/xxx`.

## Case Study
