#include<iostream>
#include<vector>
#include<queue>
#include<stack>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<list>
#include<math.h>
#include<ctype.h>
#include<map>
#include <cstring>

using namespace std;







#define NELEMS(array) (sizeof(array)/sizeof(array[0]))
#define INVALID #
#define MAXLINESIZE 512
#define VARIABLE 1
#define CONSTANT 2
#define LIST 3
#define OPERATOR 4

char failure[]="$#";
//Remember that map returns 0 if it has no mapping...!!!!! if map is of <XX,int>
/*Command line Flags*/
bool DEBUG1=false;
bool Occur_Check_Flag = true;       //by default we run occur_check() function unless we disable it from command line
bool Trace_Flag       = false;      // With this flag set , you should output a description of each call to Unify and Unify-Var.
bool Test1 = false;
bool Test2 = false;
bool Test3 = false;
bool Test4 = false;
bool Test5 = false;
bool Test6 = false;
bool Test7 = false;
bool Test8 = false;
bool Test9 = false;
bool Test10 = false;
bool DEFAULT = true;
	

const char Delim[2]=":";
const char ListDelim[2] = "$"; //this is needed because ARGS has to be sent as list.. so update this in map as Litmap["$"] = LIST





int readtextfile(char *filename);
void buildarrays();
void buildmap();
char* UNIFY(char*,char*,char*);
char* UNIFY_VAR(char*,char*,char*);
int updatetheta(char tempsubst[],char alltokens[][MAXLINESIZE],char tokvar[][MAXLINESIZE],char tokval[][MAXLINESIZE]);
int getvarval(char tempsubst[],char alltokens[][MAXLINESIZE],char tokvar[][MAXLINESIZE],char tokval[][MAXLINESIZE]);
bool OCCUR_CHECK(char *var,char *x);
int gettokens(char expression[],char tokens[][MAXLINESIZE],const char Delimiter[]);
void getargs(int totalx,char tokensx[][MAXLINESIZE],string&, string&/*char FIRSTARGSX[],char RESTARGSX[]*/);
void display(char x[],char y[], char subst[]);
void displayUNIVAR(char var[],char x[],char subst[]);

struct ltstr
{
  bool operator()(const char* s1, const char* s2 ) const
  {
    return strcmp(s1, s2) < 0;
  }
};


//ALL GLOBALS VARS
map<const char*,int, ltstr> Litmap;

map<const char*,const char*> thetamap;
char InputLines[MAXLINESIZE][MAXLINESIZE];
int numoflines=0;


int numoflists=0;
char LISTS[MAXLINESIZE][MAXLINESIZE];

int numofvars=0;
char VARS[MAXLINESIZE][MAXLINESIZE];

int numofconsts=0;
char CONSTS[MAXLINESIZE][MAXLINESIZE];

int numofops=0;
char OPS[MAXLINESIZE][MAXLINESIZE];

char lhs[MAXLINESIZE]; //space seperated like < a + b
char rhs[MAXLINESIZE]; //space seperated
char sub[MAXLINESIZE]=""; //substitution string.

/*
char lhstoken[MAXLINESIZE][MAXLINESIZE];  //has tokens
char rhstoken[MAXLINESIZE][MAXLINESIZE];  //has tokens
*/







 








int main(int argc, char * argv[])
{
    char filename[256];
    switch(argc)
    {
         case 1:
            printf("ERROR:  No arguments specified: \n");
            printf("USAGE: ./CS561P2.exe [FLAGS] FILENAME\n");
            printf("USAGE: ./CS56P2.exe [FLAGS] FILENAME\n");
            printf("EG:    ./CS561P2.exe -trace -noErrorCheck FILENAME\n");
            exit(0);
            break;

        default: 
            int FLAG = 0;
            for(int i= 0; i< argc-1; i++)
            {
                if(!strcmp(argv[i],"-trace")   || !strcmp(argv[i],"-TRACE") )  {Trace_Flag = true;FLAG++;}
                if(!strcmp(argv[i],"-noOccurCheck")   || !strcmp(argv[i],"-NOOCCURCHECK") || !strcmp(argv[i],"-nooccurcheck"))  {Occur_Check_Flag=false;FLAG++;}
                if(!strcmp(argv[i],"-d")){DEBUG1=true;}
        if(!strcmp(argv[i],"-t1")){Test1=true;}
        if(!strcmp(argv[i],"-t2")){Test2=true;}
        if(!strcmp(argv[i],"-t3")){Test3=true;}
        if(!strcmp(argv[i],"-t4")){Test4=true;}
        if(!strcmp(argv[i],"-t5")){Test5=true;}
        if(!strcmp(argv[i],"-t6")){Test6=true;}
        if(!strcmp(argv[i],"-t7")){Test7=true;}
        if(!strcmp(argv[i],"-t8")){Test8=true;}
        if(!strcmp(argv[i],"-t9")){Test9=true;}
        if(!strcmp(argv[i],"-t10")){Test10=true;}
	 if(!strcmp(argv[i],"-D")){DEFAULT=true;}

            } 
            strcpy(filename,argv[argc-1]);  //Last argument will be filename
      
            if(DEBUG1) /* *******************************************************************************/
            {
                printf("DEBUGFLAG = %d\n",DEBUG1);
                printf("TRACEFLAG = %d\n",Trace_Flag);
                printf("OCCURCHECK = %d\n",Occur_Check_Flag);
                printf("filename:%s\n",filename);
            }
            break;
    }



    readtextfile(filename);
    buildarrays();
    buildmap();
    char output[MAXLINESIZE] = "";
 
    char x[MAXLINESIZE]="";
    char y[MAXLINESIZE]="";
   
    strcpy(x,lhs); //default
    strcpy(y,rhs); //default         


    if(Test1){cout<<"Test1:TO TEST FAILURE/n";strcpy(x,"a");strcpy(y,"b");strcpy(sub,failure);strcpy(output,UNIFY(x,y,sub)); cout<<"X:"<<x<<endl; cout<<"Y:"<<y<<endl; cout<<"Sub:"<<sub<<endl;    }
    if(Test2){cout<<"Test2:TO TEST var==var/n";strcpy(x,"x");strcpy(y,"x");strcpy(sub,"x==y test");strcpy(output,UNIFY(x,y,sub));cout<<"X:"<<x<<endl; cout<<"Y:"<<y<<endl; cout<<"Sub:"<<sub<<endl;    }
    if(Test3){cout<<"Test3:UNIFY_VAR CASE (var,x,sub)/n";strcpy(x,"x");strcpy(y,"x");strcpy(sub,"x:10 y:20 z:30");/*UNIFY_VAR(x,y,sub);*/strcpy(output,UNIFY_VAR(x,y,sub));cout<<"X:"<<x<<endl; cout<<"Y:"<<y<<endl; cout<<"Sub:"<<sub<<endl;    }
     //testing unify_var also tests that map doesnt take two entries if u uncomment the above unifyvar call.



if(Test4)
{
   cout<<"Test4- UNIFY_VAR TEST/n";
   char testtheta[MAXLINESIZE] ="x:10 y:20 z:30";
   char testtokens[MAXLINESIZE][MAXLINESIZE];
   char testvar[MAXLINESIZE][MAXLINESIZE];
   char testval[MAXLINESIZE][MAXLINESIZE];
   int totaltokens = getvarval(testtheta,testtokens,testvar,testval);
  
   if(DEBUG1)
   {
	    for(int i=0;i<totaltokens;i++)
	    {
		cout<<"testvars :"<<testvar[i]<<"="<<testval[i]<<endl;
	    }
   }
   char tm[]="<a s >";
   strcpy(output,UNIFY_VAR(testvar[0],tm,testtheta)); //don pass nething else other than testvar[0] since map will fail..

  cout<<"X:"<<testvar[0]<<endl;
  cout<<"Y:"<<"<a s >"<<endl;
  cout<<"sub:"<<testtheta<<endl;
}
 
if(Test5)
{
   cout<<"Test5:variable not found in subst string test/n";
   char testtheta[MAXLINESIZE] ="x:10 y:20 z:30";
   char testtokens[MAXLINESIZE][MAXLINESIZE];
   char testvar[MAXLINESIZE][MAXLINESIZE];
   char testval[MAXLINESIZE][MAXLINESIZE];
   int totaltokens = getvarval(testtheta,testtokens,testvar,testval);
  
   if(DEBUG1)
   {
    for(int i=0;i<totaltokens;i++)
    {
        cout<<"testvars :"<<testvar[i]<<"="<<testval[i]<<endl;
    }
   }
   char tm[]="k";
   strcpy(output,UNIFY_VAR(tm,testvar[1],testtheta)); //don pass nething else other than testvar[0] since map will fail..

  cout<<"Y:"<<testvar[1]<<endl;
  cout<<"X:"<<"k"<<endl;
  cout<<"sub:"<<testtheta<<endl;
}

   

if(Test6) //occur check test
{
   cout<<"Test5:OCCUR_CHECK TEST/n";
   char testtheta[MAXLINESIZE] ="x:10 y:11 sd:123 d:12";
   char testtokens[MAXLINESIZE][MAXLINESIZE];
   char testvar[MAXLINESIZE][MAXLINESIZE];
   char testval[MAXLINESIZE][MAXLINESIZE];
   int totaltokens = getvarval(testtheta,testtokens,testvar,testval);
   char a[100] ="< sa da sd a L1 [ a b c ] y>";
  if(OCCUR_CHECK(testvar[2],a))
   {
    cout<<testvar[2]<<" found in"<<a<<endl;
   }
   else
   {
    cout<<testvar[2]<<" not found in"<<a<<endl;
   }



}

if(Test7)
{
	cout<<"Test7:append new assignment to substitution/n";
        strcpy(x,"x");
        strcpy(y,"9 a c");
        strcpy(sub,"k:10 m:20 p:30");
        strcpy(output,UNIFY_VAR(x,y,sub));
        cout<<"X:"<<x<<endl;
        cout<<"Y:"<<y<<endl;
        cout<<"Sub:"<<sub<<endl;

}



if(Test8)
{
    strcpy(x,"x");
    strcpy(y,"10");
    strcpy(sub,"x:10");
    strcpy(output,UNIFY(x,y,sub));
    cout<<"X:"<<x<<endl;
    cout<<"Y:"<<y<<endl;
    cout<<"Sub:"<<sub<<endl;
}


if(Test9)
{
    cout<<"TEST TO CHECK FIRSTARGSX AND LASTARGSX"<<endl;
    //strcpy(x,"+ ( $ ( + ( x y + ( z k ) ) $ ( a b ) )");
    //strcpy(x,"+ ( $ ( + ( x y + ( z k ) )");
    //strcpy(x,"+ ( $ ( a b ) + ( a d ) )");
    strcpy(x,"$ ( a b ) ");
    strcpy(y,"$ ( 1 2 ) ");
    strcpy(sub,"");
    cout<<"*************INPUT********************************"<<endl;
    cout<<"X:"<<x<<endl;
    cout<<"Y:"<<y<<endl;
    cout<<"Sub:"<<sub<<endl;
    cout<<"***********TRACE********************************"<<endl;
    strcpy(output,UNIFY(x,y,sub));

}



//DEFAULT CASE with file input   
if(DEFAULT)
{
	    //cout<<"***********TRACE********************************"<<endl;
	    strcpy(output,UNIFY(x,y,sub));

	    //cout<<"*************INPUT********************************"<<endl;
	    cout<<"LHS:"<<x<<endl;
	    cout<<"RHS:"<<y<<endl;
	    //cout<<"Substitution:"<<sub<<endl;

	   // cout<<"**************************************************"<<endl;	
}
   // cout<<"***************OUTPUT****************************"<<endl;
    if(!strcmp(output,failure))
        cout<<"NO SUBSTITUTION POSSIBLE"<<endl;
    else
        cout<<"Substitution:{"<<output<<" }"<<endl;
  //   cout<<"*******************************************"<<endl;
  
return 1;
} //end of main




 

char* UNIFY(char *x,char *y,char* subst)
{
        int i;
 	if(Trace_Flag)
	{
        	display(x,y,subst);//cout<<"UNIFY(["<<x<<"], ["<<y<<"], {" << subst <<"})"<<endl;
	}
        if(!strcmp(subst,failure)){return failure;}
       // cout<<"strcmo("<<x<<" ,"<<y<<endl; 
        if(!strcmp(x,y)){return subst;}
   
        if(Litmap[x]==VARIABLE)
        {
        	if(DEBUG1){cout<<x<<" is a VARIABLE"<<endl;}
        	return UNIFY_VAR(x,y,subst);
        }
       
        if(Litmap[y]==VARIABLE)
    	{
        	if(DEBUG1){cout<<y<<" is a VARIABLE"<<endl;}
        	return UNIFY_VAR(y,x,subst);
   	}

         //lets start parsing x and y
   
    int totalx = 0;
    char tokensx[MAXLINESIZE][MAXLINESIZE];
    const char delimiter[2]=" ";
    totalx = gettokens(x,tokensx,delimiter);
    int totaly = 0;
    char tokensy[MAXLINESIZE][MAXLINESIZE];
    totaly = gettokens(y,tokensy,delimiter);

         //only if all 4 conditions satisfy do we go ahead with considering the expression as compound.. for eg:lhs= 'op2'  rhs='op1' will fail
        if(Litmap[tokensx[0]]==OPERATOR && Litmap[tokensy[0]] == OPERATOR && totalx>1 && totaly>1)
        {
		
		//cout<<"OPx:"<<tokensx[0]<<" OPy:"<<tokensx[0]<<"totalx:"<<totalx<<"totaly:"<<totaly<<endl;
                  char ARGSX[MAXLINESIZE]="";
                  char ARGSY[MAXLINESIZE]="";  
                  
                  string argsx("");
                  string argsy("");   
               
                 //cout<<"Compound(X)=OPERATOR: "<<tokensx[0]<<" Compound(Y)=OPERATOR: "<<tokensy[0]<<endl;
                 //cout<<"X:"<<argsx<<"  Y:"<<argsy<<endl;      
                 // cout<<"original argsx " <<x<<endl;
                 // cout<<"original argsy " <<y<<endl;
                 argsx.append(ListDelim);
                 argsy.append(ListDelim);
                 for(i=1;i<totalx;i++)
                 {
                        argsx.append(" ");
                        argsx.append(tokensx[i]);
                 }
                 for(i=1;i<totaly;i++)
                 {
                        argsy.append(" ");
                        argsy.append(tokensy[i]);
                 }
                 strcpy(ARGSX,argsx.c_str());
                 strcpy(ARGSY,argsy.c_str());

                return UNIFY(ARGSX,ARGSY,UNIFY(tokensx[0],tokensy[0],subst));
       
          
        }//end of if the first literal is an operator
       




	 /*********check if first LITERAL IS LIST: *****************/
         string firstargsx(""),firstargsy(""),restargsx(""),restargsy("");
	 char RESTARGSX[MAXLINESIZE]="";
	 char RESTARGSY[MAXLINESIZE]="";
         char FIRSTARGSX[MAXLINESIZE]="";
         char FIRSTARGSY[MAXLINESIZE]="";
         if(Litmap[tokensx[0]]==LIST && Litmap[tokensy[0]] == LIST)
         {
	
			if(totalx<=3 && totaly<=3)//empty list return subst;
			{
				//cout<<"EMPTY LIST return subst"<<endl;
				return subst;
			}
		        else    //0  1 2 3 4 5    //0 1 2 3   //0 1 2
			{	//L2 ( a b c )    //L ( a )   //L ( )
				if(totalx<=3 || totaly<=3) //if one of them is empty then return fail
				{
					return failure;
				} 
	   		}


			if(Litmap[tokensx[2]]==VARIABLE ||Litmap[tokensx[2]]==CONSTANT )
			{
					
					firstargsx = tokensx[2];
					restargsx.append(ListDelim);
					restargsx.append(" (");
					for(i=3;i<totalx;i++) 
					{
						restargsx.append(" ");
						restargsx.append(tokensx[i]);
						restargsx.append(" ");
					}
					//cout<<"VARIABLE OR A CONSTANT firstargx "<<firstargsx<<" restargsx "<<restargsx<<endl;
					
					//restargsx.append(")");
			}

			if(Litmap[tokensy[2]]==VARIABLE ||Litmap[tokensy[2]]==CONSTANT)
			{
					firstargsy = tokensy[2];
					restargsy.append(ListDelim);
					restargsy.append(" (");
					for(i=3;i<totaly;i++) 
					{
						restargsy.append(" ");
						restargsy.append(tokensy[i]);
						restargsy.append(" ");
					}
				//cout<<"VARIABLE OR A CONSTANT firstargy "<<firstargsy<<" restargsy "<<restargsy<<endl;
			}
			//restargsy.append(")");



			//cout << firstargsx <<"   "<<restargsx <<" "<<firstargsy <<"   "<<restargsy <<endl;
		        if(Litmap[tokensx[2]]==LIST || Litmap[tokensx[2]]==OPERATOR)  //if first argument is LIST OR OPERATOR:
			{
					firstargsx="";
					restargsx="";
					getargs(totalx,tokensx,firstargsx,restargsx);

				if(Test9)
				{
	

					cout<<"List/OpXfirstargsx="<<firstargsx<<endl;
					cout<<"List/OpXrestargsx ="<<restargsx<<endl;
					int dummy;
					cin>>dummy;
				}

			}


		        if(Litmap[tokensy[2]]==LIST || Litmap[tokensy[2]]==OPERATOR)  //its a LIST or an operator.:
			{
					firstargsy="";
					restargsy="";
					getargs(totaly,tokensy,firstargsy,restargsy);
				if(Test9)
				{
	

					cout<<"LIST/OPYfirstargsy="<<firstargsy<<endl;
					cout<<"LIST/OPYrestargsy ="<<restargsy<<endl;
					int dummy;
					cin>>dummy;
				}			  
					
			}
			strcpy(RESTARGSX,restargsx.c_str());	
			strcpy(RESTARGSY,restargsy.c_str());
			strcpy(FIRSTARGSX,firstargsx.c_str());
			strcpy(FIRSTARGSY,firstargsy.c_str());
			return UNIFY(RESTARGSX,RESTARGSY,UNIFY(FIRSTARGSX,FIRSTARGSY,subst));
	  }//end of check if FIRST LITERAL IS A  list
       

   //cout<<"UNIFY() UNREACHABLE CODE!!"<<endl;
   return failure;
}

// var is variable, x is expression..(but wont be variable) and sub is sub
char* UNIFY_VAR(char* var,char* x,char* subst)
{
   if(Trace_Flag)
   {
    	displayUNIVAR(var,x,subst);
   }
   //cout<<"		UNIFY_VAR:("<<var<<" , "<<x<<" , {" << subst <<"})"<<endl;
   char alltokens[MAXLINESIZE][MAXLINESIZE];
   char tokvar[MAXLINESIZE][MAXLINESIZE];
   char tokval[MAXLINESIZE][MAXLINESIZE];
   char tempsubst[MAXLINESIZE];
   strcpy(tempsubst,subst);

   int totaltokens = updatetheta(tempsubst,alltokens,tokvar,tokval);
  

   int i;
   for(i=0;i<totaltokens;i++)
   {
     if(!strcmp(var,tokvar[i])) break;
   }

   if(i==totaltokens) //meaning var doesnt yet have a value and is not in substitution set theta. so check if x/val is in theta elseif(x/val belongs theta)
   {
     if(DEBUG1){cout<<var<<" not found in theta "<<subst<<endl;}
     int j;
     for(j=0;j<totaltokens;j++)
     {
         if(!strcmp(x,tokvar[j])) break;
     }
     if(j==totaltokens)
         {
                if(DEBUG1){ cout<<x<<" not found in"<<subst<<endl;}
          	if(OCCUR_CHECK(var,x)) //function will return false if we Occur_Check_Flag is false so we wont be running occurcheck
                 {
                     cout<<"OCCUR_CHECK FAILURE!!!!!! "<<var<<"<< occurs is "<<x<<endl;
                     return failure;
                 }
		else
		 {
		    if(DEBUG1){cout<<"UpdateTheta"<<var<<Delim<<x<<"to "<<subst<<endl;}
		    string newsub(subst);
		    string newstrx("");
		    char newx[MAXLINESIZE]="";
		    strcpy(newx,x);
		    char *rpch;
		    rpch = strtok (newx," ");
		    while (rpch != NULL)
		    {
		                 //printf ("%s\n",pch);
		                  newstrx.append(rpch);
		                rpch = strtok (NULL, " ");
		    }     
		    if(strlen(subst)!=0)
		    {newsub.append(" ");} //add a space only if its not empty
		    newsub.append(var);
		    newsub.append(Delim);
		    newsub.append(newstrx);
		    if(DEBUG1){cout<<"old theta was:"<<subst<<endl;}
		    subst = new char [newsub.size()+1];
		    strcpy(subst,newsub.c_str());
		    if(DEBUG1){cout<<"new theta is :"<<subst<<endl;}
		    return subst;   
		    
		 }
         }
         else
         {   
             if(DEBUG1)cout<<"found "<<x<<" in theta whose value is "<<tokval[j]<<endl;
       
            return UNIFY(var,tokval[j],subst);
         }
   }
   else
   {
        if(DEBUG1){cout<<"UNIFY_VAR:found "<<var<<" in theta "<<subst<<" whose value is "<<tokval[i]<<endl;}
        return UNIFY(tokval[i],x,subst);
   }


cout<<"UNIFY_VAR()----UNREACHABLE CODE!!*************"<<endl;
   //return failure;
}


void displayUNIVAR(char var[],char x[],char subst[])
{

//cout<<"		UNIFY_VAR:("<<var<<" , "<<x<<" , {" << subst <<"})"<<endl;

    int total = 0;
    char tokens[MAXLINESIZE][MAXLINESIZE];
    const char delimiter[2]=" ";
    total = gettokens(var,tokens,delimiter);
    cout<<"    UNIFY_VAR:(";

    //<<var<<
    cout<<"[";
    for(int i=0;i<total;i++)
    {
      cout<<tokens[i];
    }	
    cout<<"] ";

    cout<<",";

    total = gettokens(x,tokens,delimiter);
    cout<<"[";
    for(int i=0;i<total;i++)
    {
      cout<<tokens[i];
    }	 
    cout<<"] ";

    cout<<", {";
    cout<<subst <<"})"<<endl;

}

void display(char x[],char y[],char subst[])
{
    int total = 0;
    char tokens[MAXLINESIZE][MAXLINESIZE];
    const char delimiter[2]=" ";
    total = gettokens(x,tokens,delimiter);
	
    cout<<"UNIFY([";

     for(int i=0;i<total;i++)
     {
		if(Litmap[tokens[i]]==LIST) continue;
		if(Litmap[tokens[i]]==0)
		{
			if(tokens[i][0]==')' && strlen(tokens[i])==1){cout<<"] ";}
			else if(tokens[i][0]=='(' && strlen(tokens[i])==1)cout<<"[";

			continue;
		}

		cout<<tokens[i];

		
		if(Litmap[tokens[i]]==VARIABLE ||Litmap[tokens[i]]==CONSTANT)
		{
			//if(i==(total-2) || i==(total-1))continue;

			cout<<" ";
		}
     }
    cout<<"], [";




     total = gettokens(y,tokens,delimiter);
     for(int i=0;i<total;i++)
     {
		if(Litmap[tokens[i]]==LIST) continue;
		if(Litmap[tokens[i]]==0)
		{
			if(tokens[i][0]==')' && strlen(tokens[i])==1)cout<<"]";
			else if(tokens[i][0]=='(' && strlen(tokens[i])==1)cout<<"[";
			
			continue;
		}

		cout<<tokens[i];
		if(Litmap[tokens[i]]==VARIABLE ||Litmap[tokens[i]]==CONSTANT)
		{
				//if(i==(total-2) || i==(total-1))continue;

				cout<<" ";
		}
     }


    cout<<"], {"; 
    cout<<subst; 
    cout<<"})"<<endl;
 
}

/*
0 1 2  3 4 5 6  7 8 9  
$ ( L2 ( a b )  x y )  
FIRSTARGSX = L2 ( a b )   RESTARGSX $ ( x y )
if the first token is operator or LIST... you just pull it as it is... u dont have to convert operator to LIST since it will be converted by previous functon	
*/
void getargs(int totalx,char tokensx[][MAXLINESIZE],string& fx, string& rx/*char FIRSTARGSX[],char RESTARGSX[]*/)
{
			  
				int i=0;	
				string firstargsx("");
				string restargsx("");
                                firstargsx.append(tokensx[2]);
				firstargsx.append(" ");
				firstargsx.append("(");
				//firstargsx.append(" ");
				int bracecount=1;
				i=4;
				while(i<totalx)
				{
				 	if(!strcmp(tokensx[i],"("))
						bracecount++;

					if(!strcmp(tokensx[i],")"))
					{
						bracecount--;
	         				if(bracecount==0 )
						{
							firstargsx.append(" )");
							break;	
						}
					}
				 firstargsx.append(" ");
				 firstargsx.append(tokensx[i]);
				 i++;	
				} //end of while
			//fetch the rest of the args and make it a list $ (rest of the arguments)
			restargsx.append(ListDelim);
			restargsx.append(" (");
			if(i!=totalx)
			{
				i++;
				while(i<totalx-1)
				{
					restargsx.append(" ");
					restargsx.append(tokensx[i]);
				 i++;	
				}
			}
			restargsx.append(" )");

		fx=firstargsx;
		rx=restargsx;
		if(DEBUG1)
		{
			cout<<"Ifirstargs="<<fx<<endl;
			cout<<"Irestargs ="<<rx<<endl;
		}
}




bool OCCUR_CHECK(char *var,char *expression)
{

	if(Occur_Check_Flag) //run occur check only if the flag is set else return false meaning occurcheck passed
	   {
			    int total = 0;
			    char tokens[MAXLINESIZE][MAXLINESIZE];
			    const char delimiter[2]=" ";
			    total = gettokens(expression,tokens,delimiter);

			    int i=0;
			    for(i=0;i<total;i++)
			    {
				if(!strcmp(var,tokens[i]))break;
			    }

			    if(i==total)
			    {
				return false;
			    }
			    else
			    {
				return true;
			    }
	  }
	else
  		return false;


}


int gettokens(char expression[],char tokens[][MAXLINESIZE],const char Delimiter[])
{
       int total=0;
       char tempexp[MAXLINESIZE]="";
           strcpy(tempexp,expression); 

       char *pch;
       pch = strtok(tempexp,Delimiter);
       while(pch!=NULL)
       {
        strcpy(tokens[total++],pch);
        pch=strtok(NULL,Delimiter);
       }
 

       return total;
}



int getvarval(char tempsub[],char alltokens[][MAXLINESIZE],char tokvar[][MAXLINESIZE],char tokval[][MAXLINESIZE])
{

       char tempsubst[MAXLINESIZE]="";
           strcpy(tempsubst,tempsub);  
           //create a copy so that tempsub passed doesnt get altered.

       int total=0;
       char *pch;
       pch = strtok(tempsubst," ");
       while(pch!=NULL)
       {
        strcpy(alltokens[total++],pch);
        pch=strtok(NULL," ");
       }

       for(int i=0;i<total;i++)
       {
        char *token;
        token=strtok(alltokens[i],Delim);
        while(token!=NULL)
        {
            strcpy(tokvar[i],token);
       
            token=strtok(NULL,Delim);
               
                strcpy(tokval[i],token);
       
            token =strtok(NULL,Delim);       
        }

       }
return total;
}


//This will update the map for substitution string../which currently am not using
int updatetheta(char tempsub[],char alltokens[][MAXLINESIZE],char tokvar[][MAXLINESIZE],char tokval[][MAXLINESIZE])
{

       char tempsubst[MAXLINESIZE]="";
           strcpy(tempsubst,tempsub);  

       int total=0;
       char *pch;
       pch = strtok(tempsubst," ");
       while(pch!=NULL)
       {
        strcpy(alltokens[total++],pch);
        pch=strtok(NULL," ");
       }

       for(int i=0;i<total;i++)
       {
        char *token;
        token=strtok(alltokens[i],Delim);
        while(token!=NULL)
        {
            strcpy(tokvar[i],token);
       
            token=strtok(NULL,Delim);
               
                strcpy(tokval[i],token);
       
            token =strtok(NULL,Delim);       
        }

       }
       //creating one more loop is important so that we don go and screw up the globals in case of failure string in subst
       for(int i=0;i<total;i++)
       {
        if(DEBUG1)
        {
            cout<<alltokens[i]<<endl;
        }
        if(DEBUG1)
        {
            cout<<"vars:"<<tokvar[i]<<endl;
            cout<<"vals:"<<tokval[i]<<endl;
        }
           
        if(thetamap.find(tokvar[i])==thetamap.end())
        {
            thetamap[tokvar[i]]=tokval[i];
            if(DEBUG1){cout <<"New entry added to theta:map["<<tokvar[i]<<"]="<<thetamap[tokvar[i]]<<endl;}
        }
       }

    return total;
}



void buildmap()
{


    Litmap[ListDelim]=LIST;
    if(DEBUG1){cout<<"****VARS*****"<<endl;}
    for(int i=0;i<numofvars;i++)
    {
        if(DEBUG1){printf("%s\n",VARS[i]);}
        Litmap[VARS[i]]=VARIABLE;  
      
    }
    if(DEBUG1){cout<<"****LISTS*****"<<endl;}
    for(int i=0;i<numoflists;i++)
    {
        if(DEBUG1){printf("%s\n",LISTS[i]);}
        Litmap[LISTS[i]]=LIST;  
      
    }
    if(DEBUG1){cout<<"****CONSTANTS*****"<<endl;}
    for(int i=0;i<numofconsts;i++)
    {
        if(DEBUG1){printf("%s\n",CONSTS[i]);}
        Litmap[CONSTS[i]]=CONSTANT;  
    }

    if(DEBUG1){cout<<"****OPERATOR*****"<<endl;}
    for(int i=0;i<numofops;i++)
    {
        if(DEBUG1){printf("%s\n",OPS[i]);}
        Litmap[OPS[i]]=OPERATOR;  
    }


        if(DEBUG1)
        {
        char templhs[MAXLINESIZE]="";
        char temprhs[MAXLINESIZE]=""; 

        strcpy(templhs,lhs);
        strcpy(temprhs,rhs);

            cout<<"*******LHS*****"<<endl;
            char * lpch;
            lpch = strtok (templhs," ");
            while (lpch != NULL)
            {
                //printf ("%s\n",pch);
                cout<<lpch<<"====>"<<Litmap[lpch]<<endl;
                lpch = strtok (NULL, " ");
              
            }
   
            cout<<"*****RHS*****"<<endl;
        char *rpch;
            rpch = strtok (temprhs," ");
            while (rpch != NULL)
            {
                //printf ("%s\n",pch);
                cout<<rpch<<"====>"<<Litmap[rpch]<<endl;
                rpch = strtok (NULL, " ");
            }
        }

  

}





void buildarrays()
{
    char line[MAXLINESIZE];
    strcpy(line,InputLines[0]);
    int i=0;
    while(1)
    {
        if(!strcmp(line,"@VARIABLES:"))
            {
                while(i!=numoflines  && strcmp(line,"@OPERATORS:") && strcmp(line,"@LISTS:") && strcmp(line,"@CONSTANTS:") && strcmp(line,"@LHS:") && strcmp(line,"@RHS:") )
                {
                  

                    if(strcmp(line,"@VARIABLES:") && line[0]!=' ' && strlen(line)!=0)
                    {char temp[MAXLINESIZE]="";strcpy(temp,line);/*Litmap[temp]="V";*/ strcpy(VARS[numofvars++],temp);if(DEBUG1){cout<<temp<<endl;} }
                    i++;
                    strcpy(line,InputLines[i]);
              
              
                }
            }

        if(!strcmp(line,"@OPERATORS:"))
            {
                while(i!=numoflines  && strcmp(line,"@VARIABLES:") && strcmp(line,"@LISTS:") && strcmp(line,"@CONSTANTS:") && strcmp(line,"@LHS:") && strcmp(line,"@RHS:") )
                {
                  
                    if(strcmp(line,"@OPERATORS:") && line[0]!=' ' && strlen(line)!=0)
                    {char temp[MAXLINESIZE]="";strcpy(temp,line);/*Litmap[temp]="O"*/;strcpy(OPS[numofops++],temp);if(DEBUG1){cout<<temp<<endl;} }
                  
                    i++;
                    strcpy(line,InputLines[i]);
              
              
                }
            }


        if(!strcmp(line,"@CONSTANTS:"))
            {
                while(i!=numoflines  && strcmp(line,"@VARIABLES:") && strcmp(line,"@LISTS:") && strcmp(line,"@OPERATORS:") && strcmp(line,"@LHS:") && strcmp(line,"@RHS:") )
                {
                  
                    if(strcmp(line,"@CONSTANTS:") && line[0]!=' ' && strlen(line)!=0)
                    {char temp[MAXLINESIZE]="";strcpy(temp,line);/*Litmap[temp]="C";*/strcpy(CONSTS[numofconsts++],temp); if(DEBUG1){cout<<temp<<endl;}}
                    i++;
                    strcpy(line,InputLines[i]);
              
              
                }
            }

        if(!strcmp(line,"@LISTS:"))
            {
                while(i!=numoflines  && strcmp(line,"@VARIABLES:") && strcmp(line,"@CONSTANTS:") && strcmp(line,"@OPERATORS:") && strcmp(line,"@LHS:") && strcmp(line,"@RHS:") )
                {
                  
                    if(strcmp(line,"@LISTS:") && line[0]!=' ' && strlen(line)!=0)
                    { char temp[MAXLINESIZE]="";strcpy(temp,line);/*Litmap[temp]="L";*/strcpy(LISTS[numoflists++],temp);if(DEBUG1){cout<<temp<<endl;} }
                    i++;
                    strcpy(line,InputLines[i]);
                }
            }


        if(!strcmp(line,"@LHS:"))
            {
                while(i!=numoflines  && strcmp(line,"@VARIABLES:") && strcmp(line,"@CONSTANTS:") && strcmp(line,"@OPERATORS:") && strcmp(line,"@LISTS:") && strcmp(line,"@RHS:") )
                {
                  
                    if(strcmp(line,"@LHS:") && line[0]!=' ' && strlen(line)!=0)
                        {
                            //cout<<line<<endl;
                            strcpy(lhs,line);
                            if(DEBUG1){printf("%s\n",lhs);}
                        }
                    i++;
                    strcpy(line,InputLines[i]);
              
              
                }
            }

        if(!strcmp(line,"@RHS:"))
            {
                while(i!=numoflines  && strcmp(line,"@VARIABLES:") && strcmp(line,"@CONSTANTS:") && strcmp(line,"@OPERATORS:") && strcmp(line,"@LISTS:") && strcmp(line,"@LHS:") )
                {
		   
                  	
				
                    if(strcmp(line,"@RHS:") && line[0]!=' ' && strlen(line)!=0 )
                        {
                            strcpy(rhs,line);
                            if(DEBUG1){printf("%s\n",rhs);}
                        }
		    //cout<<"Linenumber:i="<<i<<endl;
                    i++;
                    strcpy(line,InputLines[i]);
		    
              
              
                }
            }



        if(i==numoflines) break;

    }
/*
for(i=0;i<numoflines;i++)
cout<<InputLines[i]<<endl;
*/

}








int readtextfile(char *filename)
{

    FILE * fd;
    fd = fopen (filename,"r");
    if (fd==NULL)
    {
        printf("bad file name\n");    
        exit(0);
    }

    while(!feof(fd))
    {
        char line[MAXLINESIZE]="";
        fgets(line,MAXLINESIZE,fd);
        line[strlen(line)-2] = '\0';

        if(strlen(line)!=0)
        {  
            strcpy(InputLines[numoflines++],line);
        }
    }
//cout<<numoflines<<"-numberof lines"<<endl;
    fclose(fd);
    return 1;
}
