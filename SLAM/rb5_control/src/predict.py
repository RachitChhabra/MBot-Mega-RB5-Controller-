#!/usr/bin/env python
import sys
import rospy
import numpy as np
import csv


def predict_position(env_state, Gu):

    W = np.eye(len(env_state["mean"]))*0.001
    env_state["mean"][0:3] = env_state["mean"][0:3] + Gu
    env_state["covariance"] = env_state["covariance"] +  W 

    return env_state