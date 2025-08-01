# CS577: Robot Learning and Interaction




## How to run Colab in local?

1. Install Jupyter
~~~~bash
conda install -c conda-forge jupyterlab
~~~~

2. Jupyter Server Extension Installation
~~~~bash
pip install jupyter_http_over_ws
jupyter server extension enable --py jupyter_http_over_ws
~~~~

3. Run the Jupyter server
~~~~bash
jupyter notebook --NotebookApp.allow_origin='https://colab.research.google.com' --port=8888 --NotebookApp.port_retries=0
~~~~