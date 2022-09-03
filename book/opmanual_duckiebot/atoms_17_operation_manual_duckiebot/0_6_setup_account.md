# Setup - Account {#dt-account status=ready}

This section describes how to register your Duckietown account
and set up the Duckietown authentication token, as well as other accounts you will need.

<div class='requirements' markdown='1'>

Requires: Internet connection.

Requires: About 10 minutes.

Requires: Laptop with Duckietown Shell command installed and correctly setup.

Results: Duckietown token correctly set up.

</div>

## Sign up on the Duckietown website {#dt-account-register}

To register on the Duckietown website, go to:

> [`https://www.duckietown.org/site/register`](https://www.duckietown.org/site/register)

and click through to pick a community.

Note: If you are a student, please use your official student email address, so that you can be sorted in the right group.

## Find the Duckietown token {#dt-account-find-token}

The Duckietown Token allows to authenticate your devices to the Duckietown network.

The token is a string of letters and numbers that looks something like this:

    dt1-7vEuJsaxeXXXXX-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfSJnxzuJjWaANeMf4y6XSXBWTpJ7vWXXXX

To find your token, first [login to duckietown.org](https://www.duckietown.org/pm_login), then open the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

Note: It may take up to 5 minutes after first creating your account for a token to be generated.

## Tell the Duckietown Shell your token {#dt-account-set-token}

Use the command

    laptop $ dts tok set

and follow the prompt.

Verify your token was successfully set by running the shell command,

    laptop $ dts tok status

You should see a message like the following,

    dts :  Correctly identified as uid = ***


Also make sure to tell the "challenges API" (needed if you are going to make submissions to the challenge server)  about your docker config:

    > dts challenges config --docker-username yourusername --docker-password yourpassword


## Verify everything is correct

Before we go on, this is a checkpoint to make sure you have installed everything.

If some of these commands don’t work, please go back and fix it before continuing.

If the Docker installation went well, then you can run the following command:

    laptop $ docker run hello-world
    Hello from Docker!
    This message shows that your installation appears to be working correctly.

If you set up a Github account and private key, you should be able to run this command successfully:

    laptop $ ssh -T git@github.com
    Hi GITHUB_USERNAME! You've successfully authenticated, but GitHub does not provide shell access.
    
If you have a valid DockerHub account then you can login as follows.

    laptop $ docker login -u DOCKER_USERNAME
    Password:

If the Duckietown Shell was installed, then you can run a command like this:

    laptop $ dts version

If you correctly configured the token, then this command should work:

    laptop $ dts challenges info
    ~        You are successfully authenticated:
    ~
    ~                     ID: YOUR ID
    ~                   name: YOUR NAME
    ~                  login: YOUR DUCKIETOWN ACCOUNT 
    ~
    ~         You can find the list of your submissions at the page:
    ~
    ~              https://challenges.duckietown.org/v4/humans/users/YOUR ID

If there is something not working, please stop here. Ask for help on Stack Overflow.



## Troubleshooting

### "DTShell object has no attribute sprint" when using `dts tok set`

You have to completely reinstall `dts` and its commands. Do that by:

1. Delete the `~/.dt-shell` folder
2. Uninstall `dts` by running `pip uninstall duckietown-shell`
3. Reinstall `dts` by following the procedure in [Laptop Setup](#laptop-setup)


## Other Accounts {#dt-other-accounts}

### Github {#dt-account-github}

You will find it useful to have an acount on Github if you don't have one already. You can get one [here](https://github.com/join).
