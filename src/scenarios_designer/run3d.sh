# ! /bin/bash
config_name="my_routes.csv"
scenario_name="ControllAssessment.xml"
echo "please design the scenario at first"
python 3d_routes_and_obstacle_design.py
echo "load your configuration to scenario..."
python csv_to_xml_helper.py my_config.csv  $config_name $scenario_name
echo "load successfully"
