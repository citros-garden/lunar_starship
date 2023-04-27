# General Information
Write a few lines just to explain the project. 
You can add equations with the markdown syntax:

$$ x(t) = \int(v(t))dt + x_0 $$

# Installation
1. Clone the repository:
        ```
        git clone git@github.com:citros-garden/lunar_starship.git
        ```
2. Open folder in VSCode and reopen in Docker container


# Build
1. Build Ros2 packages:
        ```
        colcon build
        ```
2. Source the setup files:
        ```
        source install/local_setup.bash
        ```

# Run
1. Launch Ros2 node:
        ```
        ros2 launch lunar_starship launch.py
        ```
2. Open FoxGlove studio and select Rosbridge as a connection method, 
then select '/lunar_starship/state.data[index]' as an input for plot. In this case, 'index' is the index of state which data will be plotted.
OR use Ros2 listener node to receive data from '/lunar_starship/state' topic.
3. Data will be published after simulation finished