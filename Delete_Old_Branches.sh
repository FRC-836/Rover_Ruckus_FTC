git checkout master

BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [[ "$BRANCH" != "master" ]]; then
  echo 'Figure out why the program did not switch to master, then re-try "sh Delete_Old_Branches.sh"';
  exit 1;
fi

git fetch -p && for branch in `git branch -vv | grep ': gone]' | awk '{print $1}'`; do git branch -D $branch; done