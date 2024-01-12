# 人形机器人创新挑战赛仿真赛

Bipedal Robot Challenge powered by TongVerse-Lite.

## System Requirements

- Docker
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Development Environment Setup

### Setting up Docker

1. **Load the Docker Image**

   Download the image file from: `https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz`.

   Use md5sum to check the download image by run: `md5sum -c checksum.txt`. Attention: you should keep the download file which named tongverselite-release-docker_20240104.tar.gz as the sampe path of checksum.txt.

   Run the following command:
   ```bash
   docker load -i tongverselite-release-docker.tar.gz
   ```

2. **Launch the Docker Image**
   To set up the TongVerse-Lite environment, execute:
   ```bash
   bash docker-run-release.sh
   ```
   This script binds your local repository directory to `/BipedChallenge/` inside Docker and initiates a new bash shell.

## Running a Demo

Once you are in the Docker environment, you can start a demo with:

```bash
bash examples/launch_task.sh <task-id>
```
Replace `<task-id>` with an integer between 1 and 6 to select the specific demo task you want to present.

## Customzie the contoller

1. source code path: ./controller_example
2. perpare env and compile source code command : https://gitee.com/leju-robot/leju_kuavo_tongverse-lite/blob/main/controller_example/README.md
3. after creat new controller file, copy it to the folder in the docker container:`/leju_controller/`

## Submission Guidelines

### Getting Started

1. **Familiarize with Examples**
   Explore the [examples](./examples/) directory to understand how the environment works.

2. **Implement Your TaskSolver**
   Create your custom `TaskSolver` for each task in `submission/task_<id>_solver`. Please note:
   - Do not modify any code outside the `task_<id>_solver` directories.
   - Modifications to `task_launcher.py` or other core files are strictly prohibited.

### Testing Your Solution

- To test your `TaskSolver`, execute:
  ```bash
  bash submission/launch_task.sh <task-id>
  ```
  Replace `<task-id>` with an integer from 1 to 6 corresponding to the task you are testing.

### Preparing for Submission

1. **Compress Your Work**
   Compress the entire [submission](./submission/) folder.
   
   If you customize the controller source code, create a docker image which have the compile env of your controller source. Compress the docker image with readme.md (the manual to complie your controller source code), controller source code.

2. **Rename the File**
   Name the compressed file as `submission_<team-id>`. 
   If customize controller, rename the docker image, readme.md, source code compressed file to `controller_<team-id>`.

3. **Submit to the Committee**
   Send your renamed files to our committee group for evaluation.
