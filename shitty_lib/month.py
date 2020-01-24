 
#Load DB

db = {"Jan": "01",
      "Feb": "02",
      "Mar": "03",
      "Apr": "04",
      "May": "05",
      "June": "06",
      "July": "07",
      "Aug": "08",
      "Sept": "09",
      "Oct": "10",
      "Nov": "11",
      "Dec": "12"}
 
def tonum(month):
    return db[month]
