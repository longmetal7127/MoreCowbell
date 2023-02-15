import os, sys, requests, json, IPython
from bs4 import BeautifulSoup

frcgametools = "https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html"
limelight = "https://limelightvision.io/pages/downloads"

def parse_limelight():
    html_doc = requests.get(limelight).text
    soup = BeautifulSoup(html_doc, 'html.parser')

    update_table = soup.select("#MainContent > div > div > div > div > table:nth-child(15)")

    tr_array = update_table[0].select("tbody > tr")

    # I dont care about limelight 3
    for i in range(1, len(tr_array)):
        a_element = tr_array[i].select("td")[0].select("a")[0]
        link = a_element["href"]
        text = a_element.text
        print(text + " " + link)

def parse_roborio():
    html_doc = requests.get(frcgametools).text

    token1 = "NI.Download.init("
    token2 = ");"
    start = html_doc.find(token1) + len(token1)
    end = start + html_doc[start:].find(token2)

    jdata = json.loads(html_doc[start:end])

    for data in jdata:
        if "includedversions-downloadpath" in data:
            print(data["difirstavailabledate"] + " " + data["includedversions-downloadpath"])
        else:
            print(data["difirstavailabledate"])

    #IPython.embed()

parse_limelight()
parse_roborio()
