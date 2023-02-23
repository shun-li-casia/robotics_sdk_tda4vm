# Robotics SDK Sphinx Doc Generation: Internal Only (Not to Release)

Generate HTML Documentation using Sphinx tool: One with the original `sphinx_rtd_theme` and another with TI's `sphinx_rtd_theme_ti` theme.

1. Setup a Python virtual environment using `requirements.txt`

2. Update `index.rst` as needed.

3. Update configuration files:
    - `conf_rtd.py` (for original `sphinx_rtd_theme`): Robo SDK version number and etc
    - `conf_ti.py` (for `sphinx_rtd_theme_ti`): Robo SDK version number and etc

4. Run the following script
```sh
./gen_html.sh
```