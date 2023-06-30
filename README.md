需要安装VRPN库，步骤：
```bash
    git clone https://github.com/vrpn/vrpn.git
    mkdir -p vrpn/build
    cd vrpn/build
    cmake ..
    make 
    make install
```

```bash
    colcon build --packages-select entity_manager_interfaces
    source install/setup.bash
```
