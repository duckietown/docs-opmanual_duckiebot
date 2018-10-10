# Version Control with Git {#preliminaries-git status=draft}

Assigned:  Andy

## Background reading

See: [Github tutorial](https://guides.github.com/activities/hello-world/)

See: [Github workflow](https://guides.github.com/introduction/Llow/)

## Installation

The basic Git program is installed using

    $ sudo apt install git

Additional utilities for `git` are installed using:

    $ sudo apt install git-extras

This include the `git-ignore` utility, which comes in handy when you have files that you don't actually want to have pushed to the remote branch (such as temporary files).


## Setting up global configurations for Git  {#howto-git-local-config}

This should be done twice, once on the laptop, and later, on the robot.

These options tell Git who you are:

    $ git config --global user.email "![email]"
    $ git config --global user.name  "![full name]"

## Git tips

### Clone a repository

To clone a repository, either copy the HTTPS or SSH link given on the repository's webpage. Then invoke following command to download the git repository onto the local computer (actual directory you are in right now).

    $ git clone git@github.com:USERNAME/REPOSITORY.git
    
If you have SSH setup properly, you can directly download it. If you are using the HTTPS then github will ask for your credentials.

### Fork a repository

To fork (creating a copy of a repository, that does not belong to you), you simply have to go to the repository's webpage dashboard and click fork on the upper right corner.

### Fetch new branches

If new branches have been pushed recently to the repository and you don't have them you can invoke a

    $ git fetch --all
    
to see all new branches and checkout to those.

### Delete branches

To delete a local branch execute (you cannot be on the branch that you are going to delete!):

    $ git branch -d ![branch-name]

To delete a remote branch you need to push the delete command:

    $ git push origin --delete ![branch-name]

## Git troubleshooting


### Problem 1: https instead of ssh:

The symptom is:

    $ git push
    Username for 'https://github.com':

Diagnosis: the `remote` is not correct.

If you do `git remote` you get entries with `https:`:

    $ git remote -v
    origin  https://github.com/duckietown/Software.git (fetch)
    origin  https://github.com/duckietown/Software.git (push)

Expectation:

    $ git remote -v
    origin  git@github.com:duckietown/Software.git (fetch)
    origin  git@github.com:duckietown/Software.git (push)

Solution:

    $ git remote remove origin
    $ git remote add origin git@github.com:duckietown/Software.git


### Problem 1: `git push` complains about upstream

The symptom is:

    fatal: The current branch ![branch name] has no upstream branch.

Solution:

    $ git push --set-upstream origin ![branch name]


## Markdown reference

Assigned: Gianmarco
