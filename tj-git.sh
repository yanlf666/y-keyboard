#/bin/bash

YEAR_MON_DAY=$(date -d "yesterday" +"%Y-%m-%d-%T-%j-%V")  



git init

git add .

git commit -m  "$YEAR_MON_DAY"

git commit -a

git remote rm origin


git remote add origin https://github.com/yanlf666/y-keyboard



git push   origin master
