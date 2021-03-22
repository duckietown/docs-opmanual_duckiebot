# Setup - Dashboard {#duckiebot-dashboard-setup status=ready}

<div class='requirements' markdown='1'>

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

</div>


This section shows how to install the Duckietown Dashboard on the Duckiebot.


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

### Video Tutorial {#init-dashboard-video}

<div figure-id="fig:howto-dashboard-setup" figure-caption="Dashboard setup tutorial.">
    <dtvideo src="vimeo:526989336"/>
</div>

### Step-by-Step Instructions {#init-dashboard-steps}

You can find your duckietown dashboard at:

    http://![YOUR_DUCKIEBOT_NAME].local/

If the above address does not work, remove the `.local` part and just do `http://![YOUR_DUCKIEBOT_NAME]/`

Note: if `.local` does not work, that means your router's default domain name is set to something else. It will be helpful if you figure out what that is. And keep in mind that any instruction later that includes `.local` should be just ignored.

#### Steps 1, 2

By default, \\compose\\ uses Google Sign-In to authenticate the users.
In Duckietown, we use authentication based on personal tokens. You should be able to
retrieve yours by visiting the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

You should notice that the first two steps already appear to be completed.
Do not worry about creating an administrator account (Step 2) for now,
a new aministrator account will be automatically created the first time we login
using a Duckietown token.

#### Step 3

**Step 3** is about configuring the dashboard.

<div figure-id="fig:compose_first_setup_step3" figure-caption="">
  <img src="compose_first_setup_step3.png" style='width: 34em'/>
</div>

You can complete this step as you please.
Feel free to update all the fields, and remember, you can always update your
choices through the page **Settings** after you authenticate
using your personal token.

When you are happy with your choices, click on **Next**.

#### Step 4

The **Step 4: Complete** tab should now be open, as shown below.

<div figure-id="fig:compose_first_setup_step5" figure-caption="">
  <img src="compose_first_setup_step5.png" style='width: 34em'/>
</div>

You can go ahead and press **Finish**.

#### First Login

If everything went as planned, the dashboard is now configured and ready to go!

You should be able to see the login page, as shown below.

<div figure-id="fig:dashboard_login_page" figure-caption="">
  <img src="dashboard_login_page.png" style='width: 34em'/>
</div>

Note: Since your dashboard does not have an administrator account yet,
the first user to login will be automatically assigned the role of
administrator. If you have multiple tokens, make sure to keep note of
which one you used for the first login.

If you have not retrieved your personal Duckietown Token as described in [](#dt-account) yet,
it is now time to do so.
Once you have your personal Duckietown token, go ahead and click on
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

As you might have noticed, the side bar to the left now shows many more pages. Some pages are
accessible by all users (e.g., Robot), others only by administrators (e.g., Settings,
Package Store).

Take your time to visit all the pages and get comfortable with the platform.
We will discuss the functionalities offered by each page in the next sections.
