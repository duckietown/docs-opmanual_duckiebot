# Secure Shell (SSH) {#preliminaries-secure-shell status=ready}

<div class='requirements' markdown="1">

Requires: Time: 5 minutes.

Results: You will know about some useful shortcuts.

</div>

Now, we will tell you about some shortcuts that you can use to
save some time.

Note: in the future you will have to debug problems, and these
problems might be harder to understand if you rely blindly on the shortcuts.

## SSH aliases {#ssh-aliases}

Instead of using

    $ ssh ![username]@![robot name].local

You can set up SSH so that you can use:

    $ ssh my-robot

During your init_sd_card process described later in the book, the command will automagically setup `~/.ssh/config` . If you are having trouble to use it, you can follow the below instructions.

To manually create an SSH alias, create a host section in `~/.ssh/config` on your laptop with the following
contents:

    Host my-robot
        User ![username]
        Hostname ![robot name].local

Here, you can choose any other string in place of "`my-robot`".


Note that you **cannot** do

    $ ping my-robot

You haven't created another hostname, just an alias for SSH.

However, you can use the alias with all the tools that rely
on SSH, including `rsync` and `scp`.
