NAME:                 Chetan Ankola
USCID:        1895488595
email:                ankola@usc.edu
Additional Information and Assumptions:
1>Accepts input text files with each line having similar format to what is given in test inputs. for eg: I assume that the line ends with \t\n there can be unpredictable failures if the format is not maintained
Note:The file although can have order of operators , variables etc in any manner
2>In the Output string am not using “,” to seperate out the individual literals, instead am just using spaces.
3>Am not using Maps for Substitution string so am using “:” as a seperator instead of “/” reason being if “/” is used for division operator my program can fail, but it wont since I have used “:” as delimiter in substitution string
4> “#$” is my (failure )symbol that represents failure and my substitution string will have {#$} in case of failure or occurcheck failure