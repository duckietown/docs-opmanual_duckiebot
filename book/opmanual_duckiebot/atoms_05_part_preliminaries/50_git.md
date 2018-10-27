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

### Fork a repository

To fork (creating a copy of a repository, that does not belong to you), you simply have to go to the repository's webpage dashboard and click fork on the upper right corner.

### Clone a repository

To clone a repository, either copy the HTTPS or SSH link given on the repository's webpage. Then invoke following command to download the git repository onto the local computer (actual directory you are in right now).

    $ git clone git@github.com:USERNAME/REPOSITORY.git
    
If you have SSH setup properly, you can directly download it. If you are using the HTTPS then github will ask for your credentials.

### Create a new branch

After you successfully cloned a repository, you may want to work on your own branch in order not to cause any chaos in the master branch. For this, you can branch out from the master or any other branches by invoking the command

    $ git checkout -b ![branch-name]
    
To see which branch you are working on you can either use both of these commands

    $ git branch
    $ git status
    
The latter provides more information on which files you might have changed, which are staged for a new commit or that you are up-to-date (everything is ok).

### Commit and Push changes

After you edited some files, you want to push your changes from the local to the remote location. In order to do so, first do a double-check on which files you have changed and if things look legitimate. Invoke

    $ git status
    
and check the output. There will be several files, that show up in red. These are files you have changed, but not yet added for a future commit. Most of the time you want to push all your changes so you add them to your commit by executing

    $ git add --all
    
If you do not want to add all files, single files can be added. Then you need to specify each single file

    $ git add ![file-path]
    
After you solved this, add a commit message to let collaborators know, what you have changed:

    $ git commit -m "![commit-message]"
    
If everything went smooth without any issues you are ready to push your changes to your branch:

    $ git push origin ![branch-name]

### Fetch new branches

If new branches have been pushed recently to the repository and you don't have them you can invoke a

    $ git fetch --all
    
to see all new branches and checkout to those.

### Delete branches

To delete a local branch execute (you cannot be on the branch that you are going to delete!):

    $ git branch -d ![branch-name]

To delete a remote branch you need to push the delete command:

    $ git push origin --delete ![branch-name]
    
### Open a pull request

If you are working on another branch than the master or if you forked a repository and want to propose changes you made into the master, you can open a so-called `pull-request`. In order to do so, press the corresponding tab in the dashboard of a repository and then press the green button `New pull request`. You will be asked which branch from which fork you want to merge.
    
## Submitting issues

If you are experiencing issues with any code or content of a repository (such as this operating manual you are reading right now), you can submit issues. For doing so go to the dashboard of the corresponding repository and press the `Issues` tab where you can open a new request. 

For example you encounter a bug or a mistake in this operating manual, please visit this [repository](https://github.com/duckietown/docs-opmanual_duckiebot/issues) to open a new issue.

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


### Problem 2: `git push` complains about upstream

The symptom is:

    fatal: The current branch ![branch name] has no upstream branch.

Solution:

    $ git push --set-upstream origin ![branch name]


## Markdown reference

Assigned: Gianmarco
