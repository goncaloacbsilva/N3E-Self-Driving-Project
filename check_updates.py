#! /usr/bin/env python3

import os
import json
import time
#Repository path

repo_path = "RAM53C/Self-driving"

#API token
token = "6285432a5f20deb2b474169320e8af304efbe4b7"

# Owner and repository names
(owner, repo) = repo_path.split('/')

refresh_rate = 120

def checker():
    last_push = 0
    while True:
        request = os.popen('perceval github --category repository '+owner+' '+repo+' --sleep-for-rate -t '+token)
        response = request.read()
        response = json.loads(response)
        push_time = response["data"]["pushed_at"]
        push_time = push_time.replace("T", "").replace("-", "").replace(":", "").replace("Z", "")
        if last_push != push_time:
            print("Detected code push")
            print("Deploying code...")
            last_push = push_time
            print("Finished Updating")
            print("Push Time: "+str(push_time))
            print("Refreshing in " + str(refresh_rate)+" seconds")
        else:
            print("No update detected, refreshing in " + str(refresh_rate)+" seconds")
        time.sleep(refresh_rate)
        
checker()
