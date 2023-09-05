# Scripts to do some parameter identification for the steering

## What is this

Set of scripts (still work-in-progress) to do parameter identification for the steering of the RC car. Hopefully it will enable us to get rid of the bicycle mode in the controller.

## Steps

- record multiple bags of the car driving in a circle, at different speeds and circle radiuses
- put them in a folder
- pass this folder as argument to `steering_param_identification.py`
- for each bag, the script will generate a plotly html and ask you to enter the start and end time of the window of interest
    - open the html
    - pass the start time of the window where the car was driving in a circle
    - pass the end time of the window where the car was driving in a circle
    - re-open the HTML to make sure that the window and calculated average values look good
    - press enter if so
- pass the generated CSV to `csv_processor.py`
- check the generated plotly HTML

## Remaining work

- figure out how to interpolate this data and use it in the controller
- make `steering_param_identification.py` more robust


## Example images

Plotly HTML from `steering_param_identification.py`

![image](images/curves_1.png)

Plotly HTML from `steering_param_identification.py` for window validation

![image](images/curves_2.png)

Plotly HTML from `csv_processor.py`

![image](images/surface_1.png)
