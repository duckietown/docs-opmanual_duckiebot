---
name: Problems with executing Docker containers on the Duckiebot
about: Use this template to report problems related to running containers on the Duckiebot

---

### Context


*What were you doing? Where were you in the manual?*

### Command line

*What command line did you run?*

     $ docker ...
     
What is the value of the `DOCKER_HOST` variable?

```
Put here output of 

    echo $DOCKER_HOST

```

###  Problem

*What did you expect?*

*What happened instead?*


### Diagnostics

#### Software diagnostics

*What other containers are running or stopped?*

```
Put here the output of: 

    $ docker -H YOURDUCKIEBOT.local ps -a -q
   
```


#### Hardware diagnostics

What does the `rpi-health` diagnostics say?
(You can find these in Portainer, or at `http://duckiebot.local:8085`.)

- [x] I don't know because I didn't look.

or 

- [ ] Error: PI is throttled
- [ ] Error: Under-voltage
- [ ] Error: Frequency capped
- [ ] Warning: PI throttling occurred in the past.
- [ ] Warning: Under-voltage occurred in the past.
- [ ] Warning: Frequency capped occurred in the past.
- [ ] None of the above


### Error report

*Please report here the **complete** error message. You can find the container log in the Portainer interface.*  

**Note that the last error is *never* the error that caused the issue. You have to SCROLL UP and find the FIRST error in the log.**



### Other notes

*Other comments?*
