# Instructions

## UTIAS MRCLAM
Place the full UTIAS MRCLAM dataset in the same directory of this README file. Dataset files can be downloaded here: http://asrl.utias.utoronto.ca/datasets/mrclam/

# Usage
## Command Line Arguments
- ```-p, --path``` { _str_ }: Parent path to the UTIAS datasets.
    - _default_: '../data/'
- ```-d, --datasets``` { _int_ }: UTIAS datasets to process.
    - _choices_: {1, 2, 3, 4, 5, 6, 7, 8, 9}
    - _default_: 9
- ```-a, --algorithms``` { _str_ }: Localization algorithms to run on each dataset.
    - _choices_: {'lscen', 'lsci', 'lsbda', 'gssci', 'gsci'}
    - _default_: 'gsci'
- ```--comm_type``` { _str_ }: The type of communication for distributive localization algorithms.
    - _choices_: ['all', 'observation', 'range', 'observation_range']
    - _default_: 'all'
- ```--comm_range``` { _float_ }: Range of communication (meters). Only useful if the communication type is "range" or "observation_range."
    - _default_: 10
- ```--comm_rate``` { _float_ }: Rate of communication (seconds). Set to 0 to default to the time precision.
    - _default_: 1
- ```--comm_prob_fail``` { _float_ }: Probability of communication failure for each algorithm, between 0 and 1.
    - _default_: 0
- ```--dt``` { _float_ }: The time precision of each iteration (seconds).
    - _default_: 0.25
- ```--offset``` { _float_ }: The offset of the execution start time in the dataset (seconds).
    - _default_: 60
- ```--duration``` { _float_ }: The execution time starting from the offset (seconds). Set to zero to use the entire dataset.
    - _default_: 240

## Example

```
python main.py --path /Users/kjchen/Documents/Repositories/git.uclalemur.com/kjchen/tro2020/v3/data/ \
               --datasets 1 3 5 6 7 \
               --algorithms lscen lsbda gsci \
               --comm_type range \
               --comm_range 5.0 \
               --comm_rate 2.5 \
               --comm_prob_fail 0.1 \
               --dt 0.05 \
               --offset 120.0 \
               --duration 240.0
```