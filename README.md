To contribute to this Repo:
## Creating a New Branch
1. Clone this repo to your local machine:

  `git clone https://github.com/andrewismoody/FRC3680_Java.git`
  
3. Create a new local branch:

  `git checkout -B My-New-Branch`
  
5. Make your code changes
6. Test your code changes
7. Add your code changes to the commit:

  `git add .`
  
9. Commit your code changes to your local branch:

  `git commit`
  
11. Enter meaningful comments that describe what changes you made, what they are expected to do, and why.
12. Push your changes to the remote repo:

`git push -u origin My-New-Branch`

14. Submit a pull request to have your branch merged into the master branch

## Rebasing
If someone else has merged changes into master after you started working on your branch, you may need to incorporate those changes into your branch before you can merge.  To do this, you will need to:
1. Fetch the current version of the master branch from the remote repo:

  `git fetch origin master:master`
  
2. Rebase your branch onto the updated master branch:

  `git rebase -i master`
  
3. Resolve any conflicts that appear during your rebase
4. Re-push your changes to the remote repo:

  `git push --force`
  
