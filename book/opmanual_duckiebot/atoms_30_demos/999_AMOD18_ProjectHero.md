# Regression testing in the context of the Duckietown project. {#regression-tests status=beta}

To facilitate uniformity across the project as much as possible of the automated regression testing should follow this template.

## Background and information {#Background-projecthero-info}

The regression testing in the Duckietown project is handled through the same engine as the AI driving olympics challenges. For those we are familiar with making a submission one half of this procedure will be self evident.

The regression testing process has two parts, a submission and a challenge. These will be explained next.

### Submission

The submission is the piece of code that will be tested. Generating a submission is simple; the generate\_submission.py script, the config folder and the submission\_template folder available in the standard Duckitown project repository are all that is required. The contents of the submission\_template folder will be explained shortly. The script looks into your config folder (explained bellow) and generates a new folder submission\_directory which contains a folder for each of the files in your configuration folder. You can then navigate to these subfolders and run make run\_regression or make run\_regression\_local to generate and send your code for testing.

#### Submission\_template
The Submission\_template folder is where you include all the information about submissions, and from which the generate\_submissions script takes the submission specifications. It has the following structure.

```
template-submission
│   Dockerfile
│   Makefile    
│   requirements.txt
│   solution.py
│   submission.yaml
```

The Dockerfile and the Makefile are standard and need not be changed unless you want to add more than your source code. The solution.py file is the entry point for the code you want to submit to the test. You can write whatever code you want into the run function of hte Solver class, but the in the end you should call cis.set\_solution\_output\_dict\(dict\) where dict is a dictionary containing all the data you want to output from your code to the evaluator. An example of this can be found bellow. 

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
We see that I am importing myalgorithm, making an instance of MyClass defined in it, accessing the input parameters from the evaluator and then calling a function of this class to generate my output data, which I then feed to the set\_solution\_output\_dict method. Now the evaluator code will be able to access the result of calling my code.

The final import file in the submission\_template is hte submission.yaml file. This one looks like this

```
challenge: Regression_test_for_lib
protocol: RT_lib
user-label: 'Solution'
user-payload: {}
```

Here the most important lines are the challenge and the protocol line. The challenge line defines which challenge your submission is associated with and should contain the exact name of the challenge. The protocol likewise needs to match the protocol of the challenge. The user-labe is for you to give your submissions a label if you so desire.

#### config

The config folder should contain a text file for each library you want to create a submission for. In this text file should be the absolute path to one level above the directory where you store your code, and on the second line it should have the name of the directory where you story your code. The generate\_submissions script takes this absolute path and combines it with the directory name to movie a copy of your code into a new submission folder.


### Challenge

The challenge is structure which gives a user's submission a numerical value, a score, which can then be used to compare with other submissions. The challenge is defined in its own repository. A template of a challenge repository can be generated using the cookiecutter here:.
The main parts of the challenge are: the evaluation folder and the challenge.yaml.

#### challenge.yaml

The challenge.yaml file defines the challenge name, metadata about the challenge \(tags,title to display on the server, which dates the challenge should be accessable, a description of the challenge\). This file also defines the associated score and the steps the challenge runs through. These can be defined as follows:

```
scoring:
  scores:
  - name: score1
    description: temp description

steps:
  step1:
    title: Scoring step
    description: ""

    timeout: 100
    evaluation_parameters:
      services:
        evaluator:
          image: _
          build:
            context: ./evaluation
        solution:
          image: SUBMISSION_CONTAINER

    features_required:
      ram_available_mb: 80
      disk_available_mb: 100
```

The top level, steps, defines that we are going to define the steps of the challenge here. The next level includes the steps themselves. In our example we only have a single step, namely the step which generates the scores for our submission. One level down we define the title, description of the step, the timeout time, the parameters to be used for evaluation and the features required. In the evaluation\_parameters we define the services, the evaluator and the solution. The simplest definition for these, and the ones we will use, are the evaluator from the dockerimage we will define in the evaluation folder and the solution image will be the submission image. Finally the features\_required defines how much ram and disk memory we need for our evaluation \(in mb\).	

The next part is the state-machine that defines how we should move step to step. In our case it is defined as follows.

```
transitions:
  - [START, success, step1]
  - [step1, success, SUCCESS]
  - [step1, failed, FAILED]
  - [step1, error, ERROR]
```

We always start at START. On each row we first define the step we begin at, then we define the action, in our case we can have success \(step is successfull\) which leads to the terminal state of SUCCESS. Similarily for failed and error. 

If you have only one step, as the simple regression test here, then you don't have to change this boilerplate.

#### The evaluation

The evaluation is found in the /evaluation folder. The important, non-boilerplate, file is the eval.py. An example of the content of this file can be found bellow.

```
#!/usr/bin/env python
import logging
import math

from duckietown_challenges import wrap_evaluator, ChallengeEvaluator, InvalidSubmission

logging.basicConfig()
logger = logging.getLogger('evaluator')
logger.setLevel(logging.DEBUG)


class Evaluator(ChallengeEvaluator):

    def prepare(self, cie):
        cie.set_challenge_parameters({'dummy':1})

    def score(self, cie):
        solution_output = cie.get_solution_output_dict()
	'''
	Define here how scoring of a solution is done.
	'''
	temp = solution_output['data']
	score = 2*temp
        cie.set_score('score1', score, 'blurb')


if __name__ == '__main__':
    wrap_evaluator(Evaluator())
```

What we care about here is the prepare and the score functions. In the prepare function we can define the input we want to give the users submission code. The simplest way to do this is to define a dictionary containing fields for all the input data and then pass this dictionary to the cie.set\_challenge\_parameters function. This dictionary can then be accessed in the solution code by calling cie.get\_challenge\_parameters. The second part is the score function. This function takes the contents of you put in the solution\_output\_dict in your submission and uses it to generate a numerical score. The numerical score is then passed to the system by calling cie.set\_score with the corresponding score field (since we only have one score we called this score1 in the challenge.yaml file) along with the actual numerical score we want to give to the submission and a text blurb related to the score.


Thus we have defined a challenge and a solution. Lets now walk through the pipeline of getting a challenge on your local server and submitting to it.

## DEMO: The regression test pipeline {#demo-projecthero-run}

What follows now is a step by step guide to setting up a local server, defining a challenge, and making a submission to this challenge.

### Step 1 (the server)
First we need to install the local server. You can get the server from 

https://github.com/duckietown/duckietown-challenges-server

Clone this repository. Running a local server requires a mysql database so navigate into your duckietown-challenges-server folder. An run.
   
    laptop $ sudo apt-get update
    laptop $ sudo apt-get install mysql-server
    laptop $ cd db
    laptop $ mysql -u root -p < init.sql

Note that your require python2 and a number of up to date packages. Next run the following commands

    laptop $ cd ..
    laptop $ python2 setup.py develop

During the setup process you might receive errors stating that you are missing python packages. if this is the case you can try running 

    laptop $ pip install *name of missing package*

If this does not work try using Duck Duck Go (not afiliated with Duckietown) to find how to update/install the missing packages. NOTE: if the process complains that you are missing duckietown-challenges or duckietown-shell; jump to step 2 and return here when directed to.

Now you should be able to start the server by running 

    laptop $ pserve --reload deployment/duckietown-server-local.ini

And now you should be able to navigate to http://localhost:6544 in your browser of choice. Open a new terminal before starting the next step.

### Step 2 (the challenge) 

Use the provided cookiecutter to create a challenge repository for your project. This is done by using the following commands:

    laptop $ pip install cookiecutter
    laptop $ cookiecutter https://github.com/duckietown/duckietown-hero-challenge-template

Fill in your projects details when prompted and you should now have a folder with the structure described in the evaluation chapter above.
You know have the template challenge folder. Navigate into this folder and into the evaluation and run:

    laptop $ pip install -r requirements.txt 

If you already have these installed you can ignore this step. If you came here from the middle of step 1 you should now go back and continuesetting up the server before returning here.

Next matter of business is to develop your evaluation/eval.py and challenge.yaml files as desrcibed above. For this demonstration you can keep them as they are.

Now you need to have your server setup. In a separate tab turn your server on. You can now run the following command to 'upload' your challenge to the local server. Navigate to the evaluation folder in your challenge repo and run:

    laptop $ DTSERVER=http://localhost:6544 make define-challenge

This might take some time. Unless there are errors your challenge is now upladed to the server. Start an evaluator for the challenge using the command.

    laptop $ DTSERVER=http://localhost:6544 dts challenges evaluator

Now you should have a running evaluator. Open a new terminal before starting on the next step.

### Step 3 (the submission)

This assumes you have your project repository from the cookiecutter template provided here:

https://github.com/duckietown/duckietown-project-template

What you need to do now fill in the template-submission inside the sub-\* folder of the project repo as described in the background. In the config file you should specify the path to the library you want to test. Note that the folder has to be single layer. Next you run:

    laptop $ make generate_submissions

Now you should have another directory inside submissions called submission\_\*library name\*. Navigate into this folder and run

    laptop $ make run_regression_local

Now you should see a submission being made to the specified challenge. If everything is correct you should now be able to following how the evaluator is evaluating you challenge either in the terminal window where the evaluator is running or on http://localhost:6544. Once the process is done you should see that you have a new entry on the leaderboard of the defined challenge over on http://localhost:6544. 

CONGRATULATIONS YOU HAVE NOW A FULLY FUNCTIONAL PIPELINE FOR TESTING!





