# CVSC: a Close-loop Vehicle Simulator For Control Based on Carla

<img align=right width=150px  src="static/images/logo.png" />

[![build](https://img.shields.io/badge/build-passing-brightgreen?style=plastic)](https://github.com/UM-Lifelong-Learning-Lab/EasyMeeting/actions)
[![Documentation](https://img.shields.io/badge/docs-passing-blue?style=plastic)](https://docs.google.com/document/d/1gVRUaGxr5uluH6ehMqdeC0KJiINGeDPv/edit#)
[![GitHub license](https://img.shields.io/badge/lisense-MIT-orange?style=plastic)](https://github.com/UM-Lifelong-Learning-Lab/EasyMeeting/blob/main/LICENSE.txt)
[![GitHub stars](https://img.shields.io/github/stars/UM-Lifelong-Learning-Lab/EZMeeting?style=plastic)](https://github.com/UM-Lifelong-Learning-Lab/EasyMeeting/stargazers)

**[ 🏗 [Github Repo](https://github.com/UM-Lifelong-Learning-Lab/EZMeeting) | 📜 [Documentation](https://docs.google.com/document/d/1gVRUaGxr5uluH6ehMqdeC0KJiINGeDPv/edit#) | 🎓 [Paper]() ]**

Welcome to EZMeeting! EZMeeting is an online video-based meeting tool, which aims to promote better engagement, careful listening, and  reflective thinking. The product provides the following capabilities:

- 🎏 **Realtime transcription with notes**: help users to catch up all key-point during online meetings.
- 📷 **Automatic summary**: an index panel for reviewing and reflecting.
- 🚀 **Automatic notification**: promote users to be more focused during the meeting.

<img src="static/images/project_overview.png" style="border-radius: 20px;">

## 🛠 Quick Start
0. Download this repository(main for local version and heroku version, master for digital ocean version)
```bash
git clone https://github.com/UM-Lifelong-Learning-Lab/EZMeeting.git
cd EZMeeting
git branch -r | grep -v '\->' | while read remote; do git branch --track "${remote#origin/}" "$remote"; done
git fetch --all
git pull --all
git branch main
```
2. Install conda/virtualenv or any environment control software, here we use conda as an example
```bash
wget -c https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh
chmod 777 Miniconda3-latest-Linux-x86_64.sh
sh Miniconda3-latest-Linux-x86_64.sh
```
2. Create a new environment 
```bash
conda create -n python=3.7 your_env_name
```
3. Activate the environment
```bash
conda activate your_env_name
```
4. Install the required python packages

```bash
pip install -r requirments.txt
```
5. Install and Start Redis Server
```bash
sudo apt update
sudo apt install redis-server
redis-server
```
7.Run the programm
```bash
python manage.py makemigrations
python manage.py migrate
python manage.py createsuperuser(to access the database, you need a superuser account)
python manage.py runserver
```
## 🏫 Documentations

More information about EZMeeting can be found in [EZMeeting Documentation](https://docs.google.com/document/d/1gVRUaGxr5uluH6ehMqdeC0KJiINGeDPv/edit#). 
Besides, the code of our [paper]() can be found in [this repo](https://github.com/UM-Lifelong-Learning-Lab/EasyMeeting).

## 📎 Citation


If you find this work useful in your project, please consider to cite it through:

```
@article{,
  title={},
  author={},
  journal={},
  year={}
}
```
[![GitHub contributors](https://img.shields.io/github/contributors/UM-Lifelong-Learning-Lab/EZMeeting?style=plastic)](https://github.com/UM-Lifelong-Learning-Lab/EZMeeting/graphs/contributors)
[![GitHub forks](https://img.shields.io/github/forks/UM-Lifelong-Learning-Lab/EZMeeting?style=plastic)](https://github.com/UM-Lifelong-Learning-Lab/EZMeeting/network)
[![GitHub issues](https://img.shields.io/github/issues/UM-Lifelong-Learning-Lab/EZMeeting?style=plastic)](https://github.com/UM-Lifelong-Learning-Lab/EZMeeting/issues)


