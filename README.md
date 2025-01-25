# Team 2480 Robot Code

> Fork of MAXSwerve C++ Template v2025.1
> See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

This project stores the robot code for team 2480.

## Getting started
- C++ knowledge is obviously good to have. <https://www.learncpp.com/>
- Set Up VSCode liveshare to collaborate during practice
To work independently:
- Clone the repository
```sh
git clone https://github.com/Team-2480/RobotCodeCpp.git
```
- Make sure you know the basics of git https://training.github.com/downloads/github-git-cheat-sheet.pdf (git is not github)

### Build & Deployment Guide

In order to build and deploy robot code it is required to have the latest version of wpilib installed. wpilib will install a specialized vscode package which is set up for development out of the box.

The fastest and easiest way to build and deploy is to use vscode.

#### VSCode

1. ctrl+shift+p
2. Search command palette
3. Use WPILib: Build Robot Code or WPILib: Deploy Robot Code

#### Command line

Alternatively team members can run gradle wrapper in the terminal line w/ ctrl+shift+~
Doing this does not offer any practical benefits unless vscode is not available.

```sh
./gradlew build
```

And to deploy

```sh
./gradlew deploy
```

Extra commands are listed with gradlew tasks

```sh
./gradlew tasks
```

## Simple Rules of Thumb For C++ Beginners


- Simple documentation describing function or class behaviors is required in the headers:
  Every function must have its basic behavior, inputs and outputs documented.
  Documentation comments go right above the function signature so IDE's can pick it up.
  Documentation for macros and helper functions isn't required but welcome.

- Header guards are required.
  >Using pragma once is easiest since it avoids name conflicts

- Follow best practices in formatting:
  VSCode has formatting built in, press ctrl+shift+i to format. If formatting fails, either C++ intellisense or clangd extensions must be installed
  Make sure to format before commiting. Without doing this commits with logic changes will also add formatting which is confusing.

- Names must be sane:
  Make names descriptive. Avoid abbreviation unless standard. Never use single letter variables except for i.

## Configuration

Make sure to change the configurations for can id's and robot dimensions or the code won't work right!
These values can be adjusted in the `Configs.h` and `Constants.h` files.

## Architecture
include directory holds headers

deploy directory holds files to put on roborio

We will use a command based design https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html

Subsystems is a place to put code controlling mechanical components. (robot interface)
Examples: Frisbee shooter, Robot arm, Ball delivery, Climbing mechanism

Commmands is a place to put code controlling robot commands. (recipes)
Examples: Shoot frisbee, Park, Drop off coral, Pick up ball.


## Dependencies

- SPARK MAX Firmware v25.0.0

### Libraries

- latest version of wpilib (https://github.wpilib.org/allwpilib/docs/release/cpp/index.html)
- REVLib v2025.0.2 (https://codedocs.revrobotics.com/cpp/namespacerev.html)

## Template Notes

Note that this is meant to be used with a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in the Hardware Client using the `Absolute Encoder` tab under the associated turning SPARK MAX devices.
