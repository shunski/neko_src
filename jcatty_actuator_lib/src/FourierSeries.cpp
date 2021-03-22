// FourierSeries.cpp implements FourierSeries.hc


FourierSeries::FourierSeries(unsigned int MaxAmplitue, unsigned int NumOfTerms)
    max_amplitude(MaxAmplitue);
    num_of_terms(NumOfTerms);
{
    int * sequence = new int [num_of_terms];
}

// Copy Constructor
FourierSeries::FourierSeries(unsigned int MaxAmplitue, unsigned int NumOfTerms) :
    max_amplitude(MaxAmplitue);
    num_of_terms(NumOfTerms);
{
    int * sequence = new int [num_of_terms];
}
