load_package report
set PROJECT_NAME $argv
project_open $PROJECT_NAME
load_report
write_report_panel -file $PROJECT_NAME.resource.html -html \
    "Fitter||Resource Section||Fitter Resource Utilization by Entity"
project_close 
