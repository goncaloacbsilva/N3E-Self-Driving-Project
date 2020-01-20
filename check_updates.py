#! /usr/bin/env python3

from perceval.backends.core.github import GitHub

#Repository path

repo_path = "RAM53C/Self-driving"

#API token
token = "6285432a5f20deb2b474169320e8af304efbe4b7"

# Owner and repository names
(owner, repo) = repo_path.split('/')

# create a Git object, pointing to repo_url, using repo_dir for cloning
repo = GitHub(owner=owner, repository=repo, api_token=token)
# fetch all issues/pull requests as an iterator, and iterate it printing
# their number, and whether they are issues or pull requests
for item in repo.fetch():
    if 'pull_request' in item['data']:
        kind = 'Pull request'
    else:
        kind = 'Issue'
    print(item['data']['number'], ':', kind)
