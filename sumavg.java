class sumavg{
	public static void main(String args[])
	{
		float sum=0;
		float avg=0;
		for(int i=1;i<=10;i++)
		{
			sum=sum+i;
		}
		avg=sum/10;
		System.out.println("SUM : "+sum);
		System.out.println("AVERAGE :"+avg);
	}
}
