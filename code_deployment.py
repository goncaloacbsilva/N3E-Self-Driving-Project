#! /usr/bin/env python3
from os import path
import os
import json
import wget

def dt_code(text):
    step1 = text.split(" ")
    code = step1[3] + month.tonum(step1[2]) + step1[1] + step1[4].replace(":", "")
    return code

def check_config_file():
    print("Checking version...")
    if path.exists("config.json"):
        return True
    else:
        return False
    
def parse_config():
    with open('config.json') as json_file:
        data = json.load(json_file)
        version = data["version"]
    return version

def write_ver(version):
    data = {}
    data['version'] = version
    with open('config.json', 'w+') as outfile:
        json.dump(data, outfile)

def get_files_commit(commits, sha):
    filenames = []
    files = []
    for commit in commits:
        if sha != 0:
            if commit.sha in sha:
                for file in commit.files:
                    if "MAIN/" in file.filename:
                        if file.filename not in filenames:
                            files.append(file)
                            filenames.append(file.filename)
        else:
            for file in commit.files:
                if "MAIN/" in file.filename:
                    if file.filename not in filenames:
                        files.append(file)
                        filenames.append(file.filename)
    return files

def prepare_dir(filepath):
    filepath = filepath.split("/")
    last_path = ""
    for i in range(0, len(filepath)):
        if filepath[i] == filepath[-1]:
            if path.exists(last_path+filepath[i]): #If file exists replace it
                os.remove(last_path+filepath[i])
            break
        if not path.exists(last_path+filepath[i]):
            print("Creating directory: " + last_path+filepath[i])
            os.mkdir(last_path+filepath[i])
        last_path += filepath[i] + "/"

def update_calc(commits, current_ver): #How many versions do we need to parse
    cmts = []
    for commit in commits:
        if commit.sha == current_ver:
            break
        else:
            cmts.append(commit.sha)
    return cmts

def read_changelog(g, repo_path):
    repo = g.get_repo(repo_path)
    commits = repo.get_commits()
    last_commit = commits[0]
    code = last_commit.sha
    if not check_config_file():
        print("Reading Src Repository...")
        files = get_files_commit(commits, 0)
        this_ver = "NULL (Fresh Install)"
    else:
        this_ver = parse_config()
        needed_commits = update_calc(commits, this_ver)
        print("Reading File Changes... (Changelog) (Commit: "+code+")")
        files = get_files_commit(commits, needed_commits)
    print("Files to change:")
    for file in files:
        print(file.filename)
    print("Repository Version: " + code)
    print("This version: " + this_ver)
    write_ver(code)
    print("=============")
    return files, repo
        
        
def updater(files, repo):
    n_up = 0
    for file in files:
        file_link = repo.get_contents(file.filename).download_url
        print("Downloading " + file.filename + " from " + file_link)
        prepare_dir(file.filename)
        wget.download(file_link, file.filename)
        n_up += 1
    print("=============")
    print("Deployment complete!")
    print(str(n_up) + " files updated")
    

def deploy(g, repo_path):
    files, repo = read_changelog(g, repo_path)
    updater(files, repo)
