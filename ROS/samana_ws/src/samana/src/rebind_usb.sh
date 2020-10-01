#!/bin/bash
echo "Rebinding PCI 0000:00:14.0"  
sh -c 'echo -n 0000:00:14.0 > /sys/bus/pci/drivers/xhci_hcd/unbind'
sh -c 'echo -n 0000:00:14.0 > /sys/bus/pci/drivers/xhci_hcd/bind'
echo "Done rebinding PCI 0000:00:14.0"  
