# YouBot
[![python](https://img.shields.io/badge/python-v3.7.X-green.svg)](https://www.python.org/)
[![pip](https://img.shields.io/badge/pip-v10.0.X-yellow.svg)](https://pypi.org/project/pip/)
[![virtualenv](https://img.shields.io/badge/virtualenv-v15.1.X-red.svg)](https://virtualenv.pypa.io/en/stable/)

ROS packages for the KUKA YouBot robot.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Quickstart](#quickstart)
- [Contributing](#contributing)
- [Further reading / Useful links](#further-reading--useful-links)

## Requirements

Before you begin, ensure you have met the following requirements:
* You have a Linux machine with Ubuntu Xenial 16.0. LTS
* You have installed [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
* You have installed [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
* You have installed [Python 2.7](https://www.python.org/downloads/)

**Maintainer:** [Miguel Cuartin Ordaz](https://www.linkedin.com/in/macuartin/)
  
## Installation

Go to your ROS working directory:

```bash
cd ~/catkin_ws/src
```

Clone the required repositories:

```bash
git clone https://github.com/macuartin/youbot.git
```

Compile your ROS workspace:

```bash
cd ~/catkin_ws && catkin_make
```

## Quickstart

Be sure to always source the appropriate ROS setup file, e.g:

```bash
source ~/catkin_ws/devel/setup.bash
```
You might want to add the line above to your ~/.bashrc file.

Try the following command:

```bash
roslaunch youbot_bringup main.launch
```

## Contributing

To contribute, follow these steps:

1. Fork this repository.
2. Create a branch: `git checkout -b <branch_name>`.
3. Make your changes and commit them: `git commit -m '<commit_message>'`
4. Push to the original branch: `git push origin <project_name>/<location>`
5. Create the pull request.

Alternatively see the GitHub documentation on [creating a pull request](https://help.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request).

## Further reading / Useful links

* Lorem ipsum dolor sit amet, consectetur adipiscing elit.
* Lorem ipsum dolor sit amet, consectetur adipiscing elit.

## Contact

If you want to contact me you can reach me at [macuartin@gmail.com](mailto:macuartin@gmail.com)

## License
<!--- If you're not sure which open license to use see https://choosealicense.com/--->

This project uses the following license: [<license_name>](<link>).
