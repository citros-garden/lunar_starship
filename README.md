# General Information
Write a few lines just to explain the project. 
You can add equations with the markdown syntax:

$$ x(t) = \int(v(t))dt + x_0 $$

# Installation
1. Clone the repository:
```bash 
git clone git@github.com:citros-garden/lunar_starship.git
```
2. Open folder in VSCode and reopen in Docker container


# Build
1. Build Ros2 packages:
```bash 
colcon build
```
2. Source the setup files:
```bash 
source install/local_setup.bash
```

# Run
1. Launch Ros2 node:
```bash 
ros2 launch lunar_starship launch.py
```

2. Open FoxGlove studio and select Rosbridge as a connection method, 
then select '/lunar_starship/state.data[index]' as an input for plot. In this case, 'index' is the index of state which data will be plotted.
OR use Ros2 listener node to receive data from '/lunar_starship/state' topic.
3. Data will be published after simulation finished








```bash
# if building from linux machine
docker build -t lunar_starship . 
# *** when building from MAC M1 chip add FROM --platform=linux/amd64 ***
docker buildx build --platform linux/amd64 -t lunar_starship .   

# login to citros
citros login
# sync
citros sync 
# login with docker
citros docker-login

# local
docker tag citros-web localhost:5001/lulav/lunar_starship
docker push localhost:5001/lulav/lunar_starship

# upload to google artifact registry
docker tag lunar_starship us-central1-docker.pkg.dev/citros/lulav/lunar_starship
docker push us-central1-docker.pkg.dev/citros/lulav/lunar_starship

```