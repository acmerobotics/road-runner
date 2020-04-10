start /wait gradlew shadowJar
dir core\build\*.jar 
copy core\build\*.jar  ..\road-runner-quickstart\teamcode