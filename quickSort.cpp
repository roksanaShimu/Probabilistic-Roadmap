#include<iostream>
#include<conio.h>

using namespace std;
void FindPos(int a[], int begin, int end);
int main(){

	int a[6]={21, 25, 18, 2, 36, 3};

	FindPos(a,0,5);
	cout<<a[0]<<"   "<<a[1]<<"   "<<a[2]<<"   "<<a[3]<<"   "<<a[4]<<"   "<<a[5]<<"   "<<endl;
	getch();
	return 0;
}
void FindPos(int a[], int begin, int end){
	cout<<"begin: "<<begin<<"  end: "<<end<<endl;
	int r=a[begin];
	a[begin]=a[end];
	int i=begin; 
	int temp,j;
	for(j=begin;j<end-1;j++){
		if(r>a[j]){
			temp=a[i];
			a[i]=a[j];
			a[j]=temp;
			i++;
		}
	}
	temp=a[i];
	a[i]=r;
	a[end]=temp;
	if((begin+1)<=(i-1))FindPos(a,begin,i-1);
	if((i+1+1)<=(end)){
		cout<<"before: "<<a[0]<<"   "<<a[1]<<"   "<<a[2]<<"   "<<a[3]<<"   "<<a[4]<<"   "<<a[5]<<"   "<<endl;
		FindPos(a,i+1,end);
		cout<<"after: "<<a[0]<<"   "<<a[1]<<"   "<<a[2]<<"   "<<a[3]<<"   "<<a[4]<<"   "<<a[5]<<"   "<<endl;
	}
}