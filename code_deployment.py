#! /usr/bin/env python3


def dt_code(text):
    step1 = text.split(" ")
    code = step1[3] + month.tonum(step1[2]) + step1[1] + step1[4].replace(":", "")
    return code

def check_version():
    print("Checking version...")
    return 0

def read_changelog(g, repo_path):
    repo = g.get_repo(repo_path)
    if check_version() == 1:
        print("Reading Src Repository...")
        files = repo.get_contents("")
    else:
        last_commit = repo.get_commits()[0]
        code = last_commit.sha
        print("Reading File Changes... (Changelog) (Commit: "+code+")")
        files = last_commit.files
    print("Files to change:")
    for file in files:
        print(file.filename)
    print("=============")
    return files, repo
        
        
def updater(files, repo):
    n_up = 0
    for file in files:
        file_link = repo.get_contents(file.filename).download_url
        print("Downloading " + file.filename + " from " + file_link)
        #wget.download(file_link, file.filename)
        n_up += 1
    print("=============")
    print("Deployment complete!")
    print(str(n_up) + " files updated")
    

def deploy(g, repo_path):
    files, repo = read_changelog(g, repo_path)
    #updater(files, repo)
