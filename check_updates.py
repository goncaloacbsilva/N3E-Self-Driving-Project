#! /usr/bin/env python3
from github import Github
import time
import progressbar
import wget
import shitty_lib.month as month
import code_deployment as cdp

#Repository path

repo_path = "RAM53C/Self-driving"

#API token
token = "6285432a5f20deb2b474169320e8af304efbe4b7"

g = Github("6285432a5f20deb2b474169320e8af304efbe4b7")
# Owner and repository names
(owner, repo) = repo_path.split('/')

refresh_rate = 120

def refresh():
    widgets=[
        ' (', progressbar.ETA(), ') ',
    ]
    for i in progressbar.progressbar(range(100), widgets=widgets):
        time.sleep(refresh_rate/100)

def checker():
    last_push = 0
    while True:
        repo = g.get_repo("RAM53C/Self-driving")
        push_time = repo.updated_at
        push_time = str(push_time).replace(" ", "").replace("-", "").replace(":", "").replace("Z", "")
        if last_push != push_time:
            print("Detected code push")
            print("Deploying code...")
            last_push = push_time
            cdp.deploy(g, repo_path)
            print("Finished Updating")
            print("Push Time: "+str(repo.updated_at))
            print("Refreshing in " + str(refresh_rate)+" seconds")
        else:
            print("No update detected, refreshing in " + str(refresh_rate)+" seconds")
        refresh()
checker()
