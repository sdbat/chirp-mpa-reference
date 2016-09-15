# chirp-mpa-reference

This shows an example Anemometer DPA (Data Processing Algorithm). Under the covers, this will consume BW data on

```
ucberkeley/sasc/+/s.hamilton/+/i.l7g/signal/raw
```

And publish data to
```
ucberkeley/anemometer/data/$vendor/$algorithm/s.anemometer/$sensor/i.anemometerdata/signal/feed"
```

# Modifying this example

Clone the repository, and change the vendor and algorithm names. Add your logic in, and then
configure the service.yml file. You will need to give it the name of your .ent file, and
your github repository (make sure you have pushed your changes to your repo).

Then run

```
spawnctl deploy --config example.yml --uri ucberkeley/sasc/spawnpoint --name "your_algorithm_name"
```

And assuming you have bosswave setup correctly, your service is now running!
