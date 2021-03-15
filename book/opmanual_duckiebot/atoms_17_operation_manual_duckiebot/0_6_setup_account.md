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

To find your token, first login to duckietown.org, then open the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

Note: It may take up to 5 minutes after first creating your account for a token to be generated.

## Tell the Duckietown Shell your token {#dt-account-set-token}

Use the command

    laptop $ dts tok set

and follow the prompt.

Then the following command should work:

    laptop $ dts challenges info

and give a similar output to:

    You are succesfully authenticated.

       name: ![Your name]
      login: ![username]

        uid: ![user ID]

## Troubleshooting

### "DTShell object has no attribute sprint" when using `dts tok set`

You have to completely reinstall `dts` and its commands. Do that by:

1. Delete the `~/.dt-shell` folder
2. Uninstall `dts` by running `pip uninstall duckietown-shell`
3. Reinstall `dts` by following the procedure in [Laptop Setup](#laptop-setup)


## Other Accounts {#dt-other-accounts}

### Github {#dt-account-github}

You will find it useful to have an acount on Github if you don't have one already. Get one [here](https://github.com/join).

### DockerHub {#dt-account-dockerhub}

You may also need an account on the DockerHub registry for some things. Get one [here](https://hub.docker.com/signup) if you don't already have one.
