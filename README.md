# Minimum-Spaning-Tree-Gurobi
<br/>
<i>Minimum Spaning Tree Solutiın with/without constraints using Gurobi Library</i>

<h3>Prerequisites:</h3>
<ul>
<li>C++ complier</li>
<li>C++ Gurobi Solver</li>
</ul>

<span id='Minimum-Spaning-Tree-Integer-Programming'>In this repository you will find two different formulation of Minimum Spaning Tree solver. These two formulation are widely used by researchers and business people to solve MST problems.</span>

<h3 id='Miller-Tucker-Zemlin-IP-title'>Miller, Tucker and Zemlin Formulation</h3>
<h6 id='FileName'>Filename: MSTP - Miller, Tucker Zemlin.cpp</h6>
<h6 id='LP-Model'>For an example model file check out: MSTP - Miller, Tucker Zemlin.lp</h6>
<p id='Miller-Tucker-Zemlin-Formulation-For-MST'>
Simply MTZ formulation first apear in 1960 and still a popular formulation to solve Travelling Salesman Problem (TSP) or Minimum Spanning Trees (MST). It uses an Integer Programming approach to solve models.
For more information you may check out the paper they published (https://dl.acm.org/citation.cfm?id=321046) and also search for new papers with 'Miller-Tucker-Zemlin Formulation' keyword.
</p>
<br/>
<h3 id='Gavish-and-Graves-IP-title'>Gavish and Graves Formulation</h3>
<h6 id='FileName2'>Filename: MSTP - Gavish, Graves.cpp</h6>
<h6 id='LP-Model2'>For an example model file check out: MSTP - Gavish, Graves.lp</h6>
<p id='Gavish-and-Graves-Integer-Programming'> Gavish and Graves first published their paper on Integer Programming solution for TSP solution on 1978 you may check out the paper via (http://hdl.handle.net/1721.1/5363), with their different approach the formulation is still a hot topic for researchers.
</p>

<p id='work-summary'>
  In my coding files, you may find a vector based approach to read Integer Programming (IP) Models from multiple *.txt files and contruct Gurobi models and run Gurobi IP Solver, then get the results for each run.<br/>One addition to formulation I use conflicting edges as constraints but not include all the constraints at once (because time complexity goes sky high). <br/>My approach is:<br/>
</p>
<ul>
  <li>Step 1: Solve model without constraint</li>
  <li>Step 2: Check current solution has any conflicting edges</li>
  <li>Step 3.1: Add constraints for conflicting pairs into model</li>
  <li>Step 3.2: Run again and Go to Step 2</li>
  <li>Step 4: Print out the results</li>
</ul>
