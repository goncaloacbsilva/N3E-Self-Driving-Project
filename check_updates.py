#! /usr/bin/env python3

# Import GitHub API
from github import Github
import time
import wget #Linux Package Downloader
import code_deployment as cdp #Code deployment module

#Repository path
repo_path = "RAM53C/Self-driving"

#Github API token
token = "6285432a5f20deb2b474169320e8af304efbe4b7"
g = Github(token)

#Interval bettween each version check
refresh_rate = 120

'''
Function: checker
Args: - repo_path: Github repository path with the format OWNER/REPOSITORY
      - g: Github API object generated with the token
      - refresh_rate: Interval bettween each repo code check in seconds
      
Description: Check for "updated_at" timestamp value and triggers code deployment when a timestamp change is detected

'''
def checker(repo_path, g, refresh_rate):
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
        time.sleep(refresh_rate)
        
        
if __name__ == '__main__':
    checker(repo_path, g, refresh_rate)
