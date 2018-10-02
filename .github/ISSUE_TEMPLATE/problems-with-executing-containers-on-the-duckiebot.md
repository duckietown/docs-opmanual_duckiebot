---
name: Problems with executing containers on the Duckiebot
about: Use this template to report problems related to running containers on the Duckiebot

---

### Context

*What were you doing? Where were you in the manual?*

### Command line

*What command line did you run?*

     $ docker ...

###  Problem

*What did you expect?*

*What happened instead?*


### Diagnostics

What does the `rpi-health` diagnostics say?
(You can find these in Portainer, or at `http://duckiebot.local:8085`.)

- [ ] I don't know because I didn't look.

or 

- [ ] Error: PI is throttled
- [ ] Error: Under-voltage
- [ ] Error: Frequency capped
- [ ] Warning: PI throttling occurred in the past.
- [ ] Warning: Under-voltage occurred in the past.
- [ ] Warning: Frequency capped occurred in the past.
- [ ] None of the above

### Error report

*Please report here the **complete** error message.*  

**Note that the last error is *never* the error that caused the issue.** You have to find the first error in the log.

You can find the container log in the Portainer interface.

### Other notes

Other comments?
