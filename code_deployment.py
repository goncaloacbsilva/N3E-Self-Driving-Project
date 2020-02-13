#! /usr/bin/env python3


from os import path
import os
import json
import wget #Linux Package Downloader

'''
Function: check_config_file
Args: None

Description: Checks if the config file exists.

'''

def check_config_file():
    print("Checking version...")
    if path.exists("config.json"):
        return True
    else:
        return False

'''
Function: parse_config
Args: None

Description: Returns the version value from the config file

'''

def parse_config():
    with open('config.json') as json_file:
        data = json.load(json_file)
        version = data["version"]
    return version

'''
Function: setup_scripts
Args: -requirements_path: Path of requirements.txt file

Description: Setup python libs specified on the requirements file

'''

def setup_scripts(requirements_path):
    print("Instaling requirements...")
    os.system("pip3 install -r " + requirements_path)

'''
Function: write_ver
Args: -version: Version SHA Code

Description: Writes version value to the config file, if the file doesn't exists a new one is created

'''

def write_ver(version):
    data = {}
    data['version'] = version
    with open('config.json', 'w+') as outfile:
        json.dump(data, outfile)

'''
Function: get_files_commit
Args: - commits: Repository Commits Object
      - sha: Can be an array of commits (SHA Code) or 0

Description: Parses all the files that suffered changes along the commits specified in the commits array. In case of sha = 0, parses all the files of the repository

'''

def get_files_commit(commits, sha):
    filenames = []
    files = []
    for commit in commits:
        if sha != 0:
            if commit.sha in sha:
                for file in commit.files:
                    if "selfdriving_car/" in file.filename:
                        if file.filename not in filenames:
                            files.append(file)
                            filenames.append(file.filename)
        else:
            for file in commit.files:
                if "selfdriving_car/" in file.filename:
                    if file.filename not in filenames:
                        files.append(file)
                        filenames.append(file.filename)
    return files

'''
Function: prepare_dir
Args: - filepath: File path

Description: Create all the directories and sub-directories where the file will be located if they are not already created. If the file already exists, the function will delete the older in order to replace it with the new one

'''

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


'''
Function: update_calc
Args: - commits: Repository Commits Object
      - current_ver: Current Version SHA Code

Description: Returns all the commits (SHA Code) that are needed to parse in order to update from the local version to the repository version

'''

def update_calc(commits, current_ver):
    cmts = []
    for commit in commits:
        if commit.sha == current_ver:
            break
        else:
            cmts.append(commit.sha)
    return cmts

'''
Function: read_changelog
Args: - g: Github API object generated with the token
      - repo_path: Github repository path with the format OWNER/REPOSITORY

Description: Compares local version with the repository version and parses all file changes ocurred bettween the 2 versions. If the local version is unknown or the config.json file is missing the function parses all the repository files in order to perform a fresh install.
Returns an array of changed files objects and the repository object

'''

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
        print(file.filename + " / " + file.status)
    print("Repository Version: " + code)
    print("This version: " + this_ver)
    write_ver(code)
    print("=============")
    return files, repo

'''
Function: updater
Args: - files: Array of file objects
      - repo: Repository object

Description: Downloads/creates and modifies the additional or (in case of fresh install) all the filesystem specified with the array of files objects

'''

def updater(files, repo):
    n_up = 0
    for file in files:
        if file.status == "removed":
            print("Removing " + file.filename)
            if path.exists(file.filename):
                os.remove(file.filename)
        else:
            file_link = repo.get_contents(file.filename).download_url
            print("Downloading " + file.filename + " from " + file_link)
            prepare_dir(file.filename)
            wget.download(file_link, file.filename)
            if "requirements.txt" in file.filename:
                setup_scripts(file.filename)
            if ".py" in file.filename:
                print("Executing: chmod +x " + file.filename)
                os.system("chmod +x " + file.filename)
        n_up += 1
    print("=============")
    print("Deployment complete!")
    print(str(n_up) + " files updated")


'''
Function: deploy
Args: - g: Github API object generated with the token
      - repo_path: Github repository path with the format OWNER/REPOSITORY

Description: CCDS Main function, reads changes on the repository files and updates the ROS Package

'''

def deploy(g, repo_path):
    files, repo = read_changelog(g, repo_path)
    updater(files, repo)
