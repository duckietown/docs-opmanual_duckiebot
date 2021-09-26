# Regression testing in the context of the Duckietown project. {#regression-tests status=beta}

To facilitate uniformity across the project as much as possible of the automated regression testing should follow this template.

## Background and information {#Background-projecthero-info}

The regression testing in the Duckietown project is handled through the same engine as the AI driving olympics challenges. For those we are familiar with making a submission one half of this procedure will be self evident.

The regression testing process has two parts, a submission and a challenge. These will be explained next.

### Submission

The submission is the piece of code that will be tested. Generating a submission is simple; the `generate_submission.py` script, the config folder and the `submission_template` folder available in the standard Duckitown project repository are all that is required. The contents of the `submission_template` folder will be explained shortly. The script looks into your config folder (explained bellow) and generates a new folder `submission_directory` which contains a folder for each of the files in your configuration folder. You can then navigate to these subfolders and run `make run_regression` or `make run_regression_local` to generate and send your code for testing.

#### Submission\_template
The `Submission_template` folder is where you include all the information about submissions, and from which the `generate_submissions` script takes the submission specifications. It has the following structure.

```
template-submission
│   Dockerfile
│   Makefile
│   requirements.txt
│   solution.py
│   submission.yaml
```

The Dockerfile and the Makefile are standard and need not be changed unless you want to add more than your source code. The solution.py file is the entry point for the code you want to submit to the test. You can write whatever code you want into the run function of hte Solver class, but the in the end you should `call cis.set_solution_output_dict(dict)` where dict is a dictionary containing all the data you want to output from your code to the evaluator. An example of this can be found bellow.

```
#!/usr/bin/env python

from duckietown_challenges import wrap_solution, ChallengeSolution, ChallengeInterfaceSolution

class Solver(ChallengeSolution):
    def run(self, cis):
        assert isinstance(cis, ChallengeInterfaceSolution)
    #here is where your code comes in. EX:
    import myalgorithm    
    myclass = myalgorithm.MyClass()
    input_from_evaluator = cis.get_challenge_parameters()
    data = {'data': myclass.run_my_code(input_from_evaluator)}
    
    #Remember to set the solution output
    cis.set_solution_output_dict(data)

#Dont change this
if __name__ == '__main__':
    wrap_solution(Solver())
```
We see that I am importing myalgorithm, making an instance of MyClass defined in it, accessing the input parameters from the evaluator and then calling a function of this class to generate my output data, which I then feed to the `set_solution_output_dict` method. Now the evaluator code will be able to access the result of calling my code.

The final import file in the `submission_template` is the `submission.yaml` file. This one looks like this

```
challenge: Regression_test_for_lib
protocol: RT_lib
user-label: 'Solution'
user-payload: {}
```

Here the most important lines are the challenge and the protocol line. The challenge line defines which challenge your submission is associated with and should contain the exact name of the challenge. The protocol likewise needs to match the protocol of the challenge. The user-labe is for you to give your submissions a label if you so desire.

#### config

The config folder should contain a text file for each library you want to create a submission for. In this text file should be the absolute path to one level above the directory where you store your code, and on the second line it should have the name of the directory where you story your code. `The generate_submissions.py` script takes this absolute path and combines it with the directory name to movie a copy of your code into a new submission folder.


### Challenge
Information on how to define a challenge can be found here\(for now\):
[https://github.com/DaRavenox/docs-AIDO/blob/master/book/AIDO/70_developers/50_define.md](https://github.com/DaRavenox/docs-AIDO/blob/master/book/AIDO/70_developers/50_define.md)
## DEMO: The regression test pipeline {#demo-projecthero-run}

What follows now is a step by step guide to setting up a local server, defining a challenge, and making a submission to this challenge.

### Step 1 (the server)
First we need to install the local server. You can get the server from

[https://github.com/duckietown/duckietown-challenges-server](https://github.com/duckietown/duckietown-challenges-server)

If the process complains that you are missing duckietown-challenges or duckietown-shell; jump to step 2 and return here when directed to.

And now you should be able to navigate to [http://localhost:6544](http://localhost:6544) in your browser of choice. Open a new terminal before starting the next step.

### Step 2 (the challenge)

Use the provided cookiecutter to create a challenge repository for your project. This is done by using the following commands:

    laptop $ pip install cookiecutter
    laptop $ cookiecutter https://github.com/duckietown/duckietown-hero-challenge-template

Fill in your projects details when prompted and you should now have a folder with the structure described in the evaluation chapter above.
You know have the template challenge folder. Navigate into this folder and into the evaluation and run:

    laptop $ pip install -r requirements.txt 

If you already have these installed you can ignore this step. If you came here from the middle of step 1 you should now go back and continuesetting up the server before returning here.

Next matter of business is to develop your `evaluation/eval.py` and `challenge.yaml` files as desrcibed above. For this demonstration you can keep them as they are.

Now you need to have your server setup. In a separate tab turn your server on. You can now run the following command to 'upload' your challenge to the local server. Navigate to the evaluation folder in your challenge repo and run:

    laptop $ DTSERVER=http://localhost:6544 make define-challenge

This might take some time. Unless there are errors your challenge is now upladed to the server. Start an evaluator for the challenge using the command.

    laptop $ DTSERVER=http://localhost:6544 dts challenges evaluator

Now you should have a running evaluator. Open a new terminal before starting on the next step.

### Step 3 (the submission)

This assumes you have your project repository from the cookiecutter template provided here:

[https://github.com/DaRavenox/duckietown-project-template](https://github.com/DaRavenox/duckietown-project-template)

What you need to do now fill in the template-submission inside the sub-\* folder of the project repo as described in the background. In the config file you should specify the path to the library you want to test. Note that the folder has to be single layer. Next you run:

    laptop $ make generate_submissions

Now you should have another directory inside submissions called `submission_*library name*`. Navigate into this folder and run

    laptop $ make run_regression_local

Now you should see a submission being made to the specified challenge. If everything is correct you should now be able to following how the evaluator is evaluating you challenge either in the terminal window where the evaluator is running or on [http://localhost:6544](http://localhost:6544). Once the process is done you should see that you have a new entry on the leaderboard of the defined challenge over on [http://localhost:6544](http://localhost:6544).

If you see an error in the server window saying that a comparison has been made between a None and a Datetime in the `db_submission.py`, found inside the src folder on the challenge-server, add the following to the
```
    if subs_timestamp == None:
    subs_timestamp = jobs_timestamp
    if jobs_timestamp == None:
    jobs_timestamp = subs_timestamp
```

in the `get_db_timestamp` function before the return statement.



CONGRATULATIONS YOU HAVE NOW A FULLY FUNCTIONAL PIPELINE FOR TESTING!





