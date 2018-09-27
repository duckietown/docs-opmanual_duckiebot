# Duckietown account {#dt-account status=ready}

Assigned: Andrea Censi

This section describes how to register for the Duckietown account
and set up the authentication token.

<div class='requirements' markdown='1'>

Requires: Internet connection.

Requires: About 10 minutes.

Results: Duckietown token correctly set up.

</div>


## Sign up on the Duckietown website {#dt-account-register}

To register on the Duckietown website, go to:

> [`https://www.duckietown.org/site/register`](https://www.duckietown.org/site/register)


Note: If you are a student, please use your official student email address, so that you can be sorted in the right group.


## Find the Duckietown token {#dt-account-find-token}

The Duckietown Token allows to authenticate your devices to the Duckietown network.

The token is a string of letters and numbers that looks something like this:

    dt1-7vEuJsaxeXXXXX-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfSJnxzuJjWaANeMf4y6XSXBWTpJ7vWXXXX

To find your token, first login to duckietown.org, then open the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)


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
