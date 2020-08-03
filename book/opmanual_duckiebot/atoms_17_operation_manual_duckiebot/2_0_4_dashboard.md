# Setting up the Dashboard {#duckiebot-dashboard-setup status=ready}

This section shows how to install the Duckietown dashboard on the Duckiebot using Docker.


<div class='requirements' markdown='1'>

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Requires: You have configured Docker communication as documented in [](#docker-setup).

Results: You have access to a web-based Dashboard for your Duckiebot.

</div>


## The \compose\ platform {#compose-platform}

\\compose\\ is a CMS (Content Management System) platform that provides functionalities
for fast-developing web applications. Custom applications are developed as external
packages that can be installed using the built-in Package Store.

The Duckiebot Dashboard is a package that you can install on your instance
of \\compose\\ running on your Duckiebot.
To make is easier for you to get started, we provide a Docker image with \\compose\\
and all the packages you need. Follow the instructions in the next step to get started.

Visit the
[official documentation page](http://compose.afdaniele.com/docs/latest/index)
for further information about \\compose\\.

## Setting up dashboard {#init-dashboard}

### Steps 1, 2

By default, \\compose\\ uses Google Sign-In to authenticate the users.
In Duckietown, we use authentication based on personal tokens. You should be able to
retrieve yours by visiting the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

Since we are not going to use Google Sign-In, you can click on **Skip**.
This will let you skip the first two steps and move straight to **Step 3**.
Do not worry about creating an administrator account (Step 2) for now,
the Duckietown package for \\compose\\ will create one for us as soon as we
authenticate for the first time using our personal token.


### Step 3

At this point, the **Step 3** tab should be open, as shown in the image below.

<div figure-id="fig:compose_first_setup_step3" figure-caption="">
  <img src="compose_first_setup_step3.png" style='width: 34em'/>
</div>

You can complete this step as you please.
Feel free to update all the fields, and remember, you can always update your
choices through the page **Settings** after you authenticate
using your personal token.

When you are happy with your choices, click on **Next**.


### Step 4

The **Step 4: Complete** tab should now be open, as shown below.

<div figure-id="fig:compose_first_setup_step5" figure-caption="">
  <img src="compose_first_setup_step5.png" style='width: 34em'/>
</div>

You can go ahead and press **Finish**.


## First Login

If everything went as planned, the dashboard is now configured and ready to go!

You should be able to see the login page, as shown below.

<div figure-id="fig:dashboard_login_page" figure-caption="">
  <img src="dashboard_login_page.png" style='width: 34em'/>
</div>

Since we skipped the first two steps of the **First Setup** of \\compose\\
([](#monitor-first-boot)), we cannot login using a Google account.
As discussed above, the Duckietown package provides its own login system,
which is based on the Duckietown personal token (Duckietoken).
If you have not retrieved yours, it is now time to do so by visiting
the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

Note: Since your dashboard does not have an administrator account yet,
the first user to login will be automatically assigned the role of
administrator. If you have multiple tokens, make sure to keep note of
which one you used for the first login.

Once you have your personal token, you can go ahead and click on
the button **Sign in with Duckietown**.
You should now see a dialog like the one shown below,

<div figure-id="fig:dashboard_login_with_duckietown_modal" figure-caption="">
  <img src="dashboard_login_with_duckietown_modal.png" style='width: 35em'/>
</div>

Copy/paste or type in your personal token and press Login.
Wait for the token to be validated, and if your token is correct, you will
be redirected to your profile page, similar to the one shown below.

<div figure-id="fig:dashboard_profile_page" figure-caption="">
  <img src="dashboard_profile_page_full.png" style='width: 35em'/>
</div>

As you might have noticed, the left hand side bar now shows many more pages. Some pages are
accessible by all users, others only by administrators (e.g., Settings,
Package Store, Debug).

Also, you might have noticed that some Duckietown-specific pages are already
there (e.g., Portainer, Mission Control, Duckietown). This is because we used
a Docker image of \\compose\\ that comes with some Duckietown packages
pre-installed.

Take your time to visit all the pages and get comfortable with the platform.
We will discuss the functionalities offered by each page in the next sections.
