from IPython.display import display, HTML

animation = """
<div style="text-align:center;">
  <img src="https://i.gifer.com/ZZ5H.gif" style="vertical-align:middle"/>
  <p>Processing...</p>
</div>
"""

# Display the animation
display(HTML(animation))

# Your long-running code here
import time
time.sleep(5)  # Simulating a long task
